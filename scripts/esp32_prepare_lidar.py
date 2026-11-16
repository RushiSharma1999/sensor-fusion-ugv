#!/usr/bin/env python3
"""Check stopped ESP32 firmware before handing serial ownership to ROS.

Default: unchanged bot_firmware_v4.ino. No nonzero CMD, raw PWM or MODE AUTO
is generated. The earlier host_lidar protocol remains an explicit alternative.
"""
import argparse
import json
import time
import serial

ZERO = b'CMD v 0.000 w 0.000\n'


class PrepareError(RuntimeError):
    pass


def send(port, payload):
    if port.write(payload) != len(payload):
        raise PrepareError('Partial serial write')


def wait_for(port, predicate, timeout, clock=time.monotonic, retry=None):
    deadline = clock() + timeout
    retry_at = clock() + 0.5
    while clock() < deadline:
        if retry is not None and clock() >= retry_at:
            retry()
            retry_at = clock() + 0.5
        raw = port.readline()
        if not raw:
            continue
        line = raw.decode('ascii', errors='replace').strip()
        result = predicate(line)
        if result is not None:
            return result
    raise PrepareError('Timed out waiting for ESP32 response; check firmware, port and USB CDC settings')


def parse_state(line):
    if not line.startswith('STATE '):
        return None
    words = line.split()
    if len(words) % 2 != 1:
        return None
    fields = dict(zip(words[1::2], words[2::2]))
    if len(fields) != (len(words)-1)//2:
        return None
    required = ('host_active', 'stop_latched', 'ok', 'pwmL', 'pwmR')
    try:
        result = {key: int(fields[key]) for key in required}
    except (KeyError, ValueError):
        return None
    if result['host_active'] not in (0, 1) or result['stop_latched'] not in (0, 1):
        return None
    return result


def read_stopped_state(port, timeout):
    send(port, ZERO)
    # This marker follows ZERO in the MCU's parser/output queue. Ignore older
    # buffered STATE records until it arrives, then request current status.
    send(port, b'DEBUG 0\n')
    wait_for(port, lambda line: True if line == 'ACK DEBUG 0' else None, timeout)
    send(port, b'STATUS\n')
    state = wait_for(port, parse_state, timeout)
    if state['ok'] != 1:
        raise PrepareError('ESP32 reports a hardware initialization fault; ARM cannot fix it')
    if state['host_active'] != 0 or state['pwmL'] != 0 or state['pwmR'] != 0:
        raise PrepareError('ESP32 did not confirm stopped motor outputs')
    return state


def prepare(port, arm=False, timeout=3.0):
    attempted_arm = False
    try:
        before = read_stopped_state(port, timeout)
        if before['stop_latched'] and not arm:
            raise PrepareError('ESP32 STOP IS LATCHED. With wheels raised and hardware stop available, '
                               'use --drive --arm --replace to explicitly release it.')
        after = before
        if arm:
            attempted_arm = True
            send(port, b'ARM\n')
            wait_for(port, lambda line: True if line == 'ACK ARM stopped' else None, timeout)
            after = read_stopped_state(port, timeout)
            if after['stop_latched']:
                raise PrepareError('ESP32 STOP remains latched after ARM')
        return {'before': before, 'after': after, 'arm_sent': attempted_arm}
    except Exception:
        # If explicit re-arm fails part way through, request a latched stop.
        if attempted_arm:
            try:
                send(port, b'STOP\n')
            except Exception:
                pass
        raise
    finally:
        # Never leave a nonzero output request, even on a timeout/error.
        try:
            send(port, ZERO)
        except Exception:
            pass


def parse_v4_debug(line):
    if not line.startswith('DBG|T:'):
        return None
    try:
        fields = dict(part.split(':', 1) for part in line.split('|')[1:])
        left, right = map(int, fields['PWM'].split('/'))
        return dict(mode=fields['MODE'], doing=fields['DOING'],
                    auto_enabled=int(fields['AUTOEN']), pwmL=left, pwmR=right)
    except (KeyError, ValueError):
        return None


def prepare_v4(port, timeout=3.0):
    """Disable onboard AUTO, verify v4 identity and healthy stopped debug state."""
    try:
        # Both reviewed firmware versions understand STOP/zero. v4's exact ACK
        # deliberately excludes the newer ACK STOP latched protocol.
        def request_identity():
            send(port, b'STOP\n')
            send(port, ZERO)
            send(port, b'HELP\n')
        request_identity()
        def identify(line):
            if line.startswith('CMD v '):
                if 'MODE AUTO' not in line or 'PWM <L> <R>' not in line or 'ARM' in line:
                    raise PrepareError('Expected original bot_firmware_v4.ino; upload it before starting')
                return True
            return None
        # Opening USB may reset the MCU: repeat only stopped identity requests
        # within the bounded startup deadline if its early input was lost.
        wait_for(port, identify, timeout, retry=request_identity)
        # Second STOP/ACK is an ordering marker after identity/possible boot.
        send(port, b'STOP\n')
        wait_for(port, lambda line: True if line == 'ACK STOP' else None, timeout)
        send(port, b'DEBUG 1\n')
        wait_for(port, lambda line: True if line == 'ACK DEBUG 1' else None, timeout)
        state = wait_for(port, parse_v4_debug, timeout)
        if state['doing'] == 'HARDWARE ERROR':
            raise PrepareError('ESP32 v4 reports a hardware initialization fault')
        if (state['mode'] != 'IDLE' or state['auto_enabled'] != 0 or
                state['pwmL'] != 0 or state['pwmR'] != 0):
            raise PrepareError('ESP32 v4 did not confirm IDLE, AUTO disabled and zero PWM')
        return {'firmware': 'v4', 'stopped': state, 'arm_sent': False}
    except Exception:
        try:
            send(port, b'STOP\n')
        except Exception:
            pass
        raise
    finally:
        try:
            send(port, ZERO)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--firmware', choices=('v4', 'host_lidar'), default='v4')
    parser.add_argument('--arm', action='store_true', help='Explicitly clear STOP latch, staying stopped')
    parser.add_argument('--timeout', type=float, default=3.0)
    args = parser.parse_args()
    if args.firmware == 'v4' and args.arm:
        parser.error('v4 has no ARM command; omit --arm')
    if not 0 < args.timeout <= 10:
        parser.error('timeout must be in (0, 10] seconds')
    try:
        # POSIX exclusive locks prevent another cooperating serial owner.
        # Match the working ROS bridge's default DTR/RTS settings. Holding
        # these inactive prevented this adapter from answering the handshake.
        with serial.Serial(args.port, baudrate=115200, timeout=.1,
                           write_timeout=.2, exclusive=True) as port:
            result = (prepare_v4(port, timeout=args.timeout) if args.firmware == 'v4'
                      else prepare(port, arm=args.arm, timeout=args.timeout))
        print('ESP32 stopped/ready: ' + json.dumps(result, sort_keys=True))
    except (PrepareError, serial.SerialException, OSError) as error:
        raise SystemExit(str(error))


if __name__ == '__main__':
    main()
