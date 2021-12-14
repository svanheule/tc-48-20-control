# SPDX-License-Identifier: GPL-3.0-only
#
# Sander Vanheule, 2021
#
# Code to control TE Technology TC-48-20 temperature controller
# Documentation at: https://tetech.com/product/tc-48-20/


import argparse
import binascii
import enum
import re
import serial
import struct
import sys
import time

##
# Base command formatting and parsing
##

REPLY_LENGTH = 8

CONTROL_MODE_COOL = 0
CONTROL_MODE_HEAT = 1

class Parameter(int, enum.Enum):
    def __new__(cls, write_cmd, read_cmd, scaling):
        if write_cmd is not None:
            value = write_cmd
        elif read_cmd is not None:
            value = read_cmd
        else:
            raise ValueError('write_cmd and read_cmd cannot both be None')
        obj = int.__new__(cls, value)
        obj._value_ = value
        obj.write_cmd = write_cmd
        obj.read_cmd = read_cmd
        obj.scaling = scaling
        return obj

    MODEL_CODE = (None, 0x00, None)
    STATUS_ALARM = (None, 0x03, None)
    TEMP_CONTROL_C = (None, 0x01, 10)
    TEMP_AUX_C = (None, 0x04, 10)
    TEMP_SET_C = (0x1c, 0x50, 10)

    BW_PROPORTIONAL = (0x1d, 0x51, 10.) # Stored in 0.1 degrees
    GAIN_INTEGRAL = (0x1e, 0x52, 100.) # Stored in 0.01 repeats/minute
    GAIN_DIFFERENTIAL = (0x1f, 0x53, 100.) # Stored in 0.01 minutes

    #CONTROL_MODE = (0x21, 0x55, None)
    OUTPUT_ENABLE = (0x30, 0x64, None)
    OUTPUT_POWER = (None, 0x02, 511)


def calc_checksum(data_bytes):
    return sum(data_bytes) % 256

def format_command(code, data):
    cmd_data = binascii.hexlify(struct.pack('>Bh', code, data))
    chksum = calc_checksum(cmd_data)
    cmd_data += binascii.hexlify(struct.pack('>B', chksum))
    return b'*' + cmd_data + b'\r'

def parse_reply(reply):
    if len(reply) != REPLY_LENGTH or reply[0] != ord(b'*') or reply[-1] != ord('^'):
        raise Exception('reply invalid')

    chksum = calc_checksum(reply[1:5])
    reply_chksum =  binascii.unhexlify(reply[5:7])[0]

    if chksum != reply_chksum:
        raise Exception('reply checksum invalid')

    if reply[1:5] == b'XXXX':
        raise Exception('command checksum invalid')

    return struct.unpack('>h', binascii.unhexlify(reply[1:5]))[0]

def send_command(port, command, value):
    """Send a command, with 'value' being a 16-bit signed integer
    For read commands, value is 0, and the return value is the retreived
    parameter.  For write commands, value is the requested value, and the
    return value is the response from the controller.
    """
    port.write(format_command(command, value))
    return parse_reply(port.read(REPLY_LENGTH))

def send_scaled_command(port, command, value, scaling=1):
    """Send a read/write command with a scaling factor. 'value' is expressed
    in normal units, and is multiplied by 'scaling' to the units expected by
    the controller.
    Replies are divided by 'scaling' to return to the expected units.
    For example, if the controller expects or reports a value in tenths of
    degrees, provide a factor of 10 here.
    """
    return send_command(port, command, int(value * scaling)) / scaling

def read_property(port, prop):
    try:
        if prop.scaling is None:
            return send_command(port, prop.read_cmd, 0)
        else:
            return send_scaled_command(port, prop.read_cmd, 0, prop.scaling)
    except Exception as e:
        print('failed to read property:', e)
        return None

def write_property(port, prop, value):
    try:
        if prop.scaling is None:
            return send_command(port, prop.write_cmd, value)
        else:
            return send_scaled_command(port, prop.write_cmd, value, prop.scaling)
    except Exception as e:
        print('failed to write property:', e)
        return None

##
# Helpers to parse script arguments
##
def parse_time(arg):
    unit_map = {
            ''  : 1,
            's' : 1,
            'm' : 60,
            'h' : 3600,
    }

    # Regex to parse things like '50', '10.', '2h', '13.37m'
    m = re.match(r'^([0-9]+(\.[0-9]*)?)([smh]?)$', arg)
    if m is None:
        raise ValueError('invalid time argument')

    return float(m.group(1)) * unit_map[m.group(3)]


parser = argparse.ArgumentParser(description='Freezing plate controller')
parser.add_argument('--port', '-p', required=True, type=str, help='Serial port')

subparsers = parser.add_subparsers(dest='sub_command', help='available sub-commands', required=True)

subparsers.add_parser('status', help='read controller status')

parser_set = subparsers.add_parser('set', help='modify controller settings')
parser_set.add_argument('--temp', '-t', type=float, help='setpoint in degrees Celcius')
parser_set.add_argument('--proportional', '-p', type=float, help='control loop proportional bandwitdh (°C)')
parser_set.add_argument('--integral', '-i', type=float, help='control loop integral gain (repeats/min)')
parser_set.add_argument('--differential', '-d', type=float, help='control loop differential gain (min)')

parser_cycle = subparsers.add_parser('cycle',
        help='Cycle between two setpoints A and B. If present, the auxiliary temperature sensor is used to monitor the environmental temperature.')
parser_cycle.add_argument('--cycles', '-n', type=int, help='Number of temperature cycles. Cycles indefinitely if omitted')
parser_cycle.add_argument('--temp-a', required=True, type=float, help='Temperature A, in degrees Celcius')
parser_cycle.add_argument('--temp-b', required=True, type=float, help='Temperature B, in degrees Celcius')
parser_cycle.add_argument('--time-a', default=0.0, type=parse_time,
        help='Time at temperature A, in seconds. Alternative formats are 1.5h, 25m, 600s')
parser_cycle.add_argument('--time-b', default=0.0, type=parse_time,
        help='Time at temperature B, in seconds. Alternative formats are 1.5h, 25m, 600s')

args = parser.parse_args()

# Convenience definitions
STATUS_OPEN_SECONDARY = 1 << 5

# Cycle state machine
class CycleState(enum.Enum):
    COOLING = 0
    COLD = 1
    HEATING = 2
    WARM = 3

def perform_cycles(port, args):
    STABLE_BAND = 0.2

    if args.temp_a > args.temp_b:
        temp_warm, temp_cold = args.temp_a, args.temp_b
        time_warm, time_cold = args.time_a, args.time_b
        cycle_state = CycleState.HEATING
    else:
        temp_warm, temp_cold = args.temp_b, args.temp_a
        time_warm, time_cold = args.time_b, args.time_a
        cycle_state = CycleState.COOLING

    ramp_to(port, args.temp_a)
    temp_target = args.temp_a

    time_zero = time.time()
    time_start = time.time()
    time_stop = time_start

    cycles_done = 0
    remaining = args.cycles
    while remaining is None or remaining > 0:
        temp_current = read_actual_temp(port)
        time_now = time.time()

        if cycle_state in {CycleState.COOLING, CycleState.HEATING}:
            if abs(temp_current - temp_target) <= STABLE_BAND:
                time_start = time_now
                if cycle_state is CycleState.COOLING:
                    temperature_point = 'cold'
                    cycle_state = CycleState.COLD
                    time_stop = time_start + time_cold
                else:
                    temperature_point = 'warm'
                    cycle_state = CycleState.WARM
                    time_stop = time_start + time_warm

                print('[{:>8d}] Reached {} temperature {}'.format(
                        int(time_now - time_zero), temperature_point, temp_target))

        elif cycle_state in {CycleState.COLD, CycleState.WARM}:
            if time.time() >= time_stop:
                if cycle_state is CycleState.COLD:
                    cycle_state = CycleState.HEATING
                    temp_target = temp_warm
                else:
                    cycle_state = CycleSate.COOLING
                    temp_target = temp_cold

                # We're done if the next state would be the initial state
                if temp_target == args.temp_a:
                    cycles_done += 1
                    print('[{:>8d}] Cycle {} finished'.format(cycles_dones))
                    if remaining is not None:
                        remaining -= 1

                # Only start ramping if we're not done yet
                if remaining is None or remaining > 0:
                    ramp_to(port, temp_target)

        if remaining is None or remaining > 0:
            print('[{:>8d}] Current temperature: {} °C'.format(int(time_now - time_zero), temp_current), end='\r')
            time.sleep(1)


##
# Commonly used calls
##

def read_actual_temp(port):
    return read_property(port, Parameter.TEMP_CONTROL_C)

def ramp_to(port, temp):
    write_property(port, Parameter.TEMP_SET_C, temp)


# From TE example script:
#    "Some customers have noticed an improved communication from the controller
#    with a small (one to four) millisecond delay between characters. This
#    delay is optional, however feel free to attempt it in case of any
#    communication problems."
# Modify the inter_byte_timeout to a value expressed in seconds (e.g. 0.004 for
# 4 ms) to configure this delay.
try:
    with serial.Serial(args.port, 115200, timeout=0.1, inter_byte_timeout=None) as port:
        if read_property(port, Parameter.MODEL_CODE) != 9613:
            raise Exception('Invalid controller')

        status = read_property(port, Parameter.STATUS_ALARM)

        if args.sub_command == 'status':
            status_bits = [
                'HIGH ALARM 1',
                'LOW ALARM 1',
                'HIGH ALARM 2',
                'LOW ALARM 2',
                'OPEN CONTROL SENSOR',
                'OPEN SECONDARY SENSOR',
                'KEYPAD VALUE CHANGE',
            ]
            print('status flags: ' + ', '.join(status_bits[b] for b in range(len(status_bits)) if (status & (1 << b))))

            print('temperature set point: {:.1f} °C'.format(read_property(port, Parameter.TEMP_SET_C)))
            primary = read_actual_temp(port)
            print('current temperature: {:.1f} °C'.format(primary))
            if not status & STATUS_OPEN_SECONDARY:
                aux = read_property(port, Parameter.TEMP_AUX_C)
                print('current secondary temperature: {:.1f} °C'.format(aux))

            if read_property(port, Parameter.OUTPUT_ENABLE) == 1:
                print('output power fraction: {:2.1f} %'.format(100 * read_property(port, Parameter.OUTPUT_POWER)))
            else:
                print('output disabled')
    
        elif args.sub_command == 'set':
            properties = {
                'temperature setpoint' : (args.temp, Parameter.TEMP_SET_C),
                'proportional bandwidth' : (args.proportional, Parameter.BW_PROPORTIONAL),
                'integral gain' : (args.integral, Parameter.GAIN_INTEGRAL),
                'differential gain' : (args.differential, Parameter.GAIN_DIFFERENTIAL),
            }

            for desc in properties:
                value, param = properties[desc]

                if value is not None:
                    if write_property(port, param, value) is not None:
                        print('{} set to {}'.format(desc, value))
                    else:
                        print('failed to set {} to {}'.format(desc, value))


        elif args.sub_command == 'cycle':
            print('Cycle {} times: {} s @ {} °C, {} s @ {} °C'.format(
                args.cycles, args.time_a, args.temp_a, args.time_b, args.temp_b)
            )

            try:
                perform_cycles(port, args)
            except KeyboardInterrupt:
                print()


        else:
            raise Exception('Unsupported sub-command {}'.format(args.sub_command))

except Exception as e:
    print('execution failed:', e)
    sys.exit(1)
