#!/usr/bin/env python3
"""Bounded serial checks for the uploaded ESP32-S3 v4 firmware.

Default --test check sends STOP and DEBUG 1, then reads telemetry; no nonzero
velocity is requested. --test wheels requires the robot secured with all
driven wheels off the floor. It requests forward, left spin, right spin,
then checks the firmware command-loss watchdog. An operator must observe
wheel direction and have the motor-power cutoff available.

This file is NOT an obstacle-avoidance controller. It never sends MODE AUTO
or raw PWM commands and does not modify firmware. Speeds are uncalibrated
command values, not measured physical speeds. ROS bridges/serial monitors
must be stopped first. Software STOP is not latched against a new CMD from
another process. Actual stopping distance cannot be established here.
"""

import argparse
from collections import deque
import os
from pathlib import Path
import signal
import sys
import time

DEFAULT_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A84021142-if00'


def parse_record(line):
    fields = {}
    for part in line.split('|')[1:]:
        key, sep, value = part.partition(':')
        if sep:
            fields[key] = value
    return fields


def port_users(port):
    """Best-effort /proc inspection; flock is also used when opening serial."""
    target = os.path.realpath(port)
    owners = []
    for item in Path('/proc').iterdir():
        if not item.name.isdigit() or int(item.name) == os.getpid():
            continue
        try:
            for fd in (item / 'fd').iterdir():
                try:
                    if os.readlink(fd) == target:
                        owners.append((item.name, (item / 'comm').read_text().strip()))
                        break
                except OSError:
                    pass
        except (OSError, PermissionError):
            continue
    return owners


class Probe:
    def __init__(self, ser):
        self.ser = ser
        self.debug = None
        self.sonars = {}
        self.lines = deque(maxlen=100)
        self.last_debug_print = 0.0
        self.last_stop_ok = False
        self.saw_pwm = False

    def send(self, command):
        self.ser.write((command + '\n').encode('ascii'))
        self.ser.flush()

    def pump(self, seconds, expected_ack=None):
        deadline = time.monotonic() + seconds
        acknowledged = False
        while time.monotonic() < deadline:
            raw = self.ser.readline(512)
            if not raw:
                continue
            line = raw.decode('ascii', errors='replace').strip()
            self.lines.append(line)
            if line == expected_ack:
                acknowledged = True
            if line.startswith('US|'):
                record = parse_record(line)
                if record.get('ID') in ('FRONT', 'LEFT', 'RIGHT', 'BACK'):
                    self.sonars[record['ID']] = record
            elif line.startswith('DBG|'):
                record = parse_record(line)
                if 'MODE' in record:
                    self.debug = record
                    try:
                        pwm = [int(x) for x in record['PWM'].split('/')]
                        self.saw_pwm |= (record['MODE'] == 'MANUAL' and any(pwm))
                    except (KeyError, ValueError):
                        pass
                if time.monotonic() - self.last_debug_print >= 0.35:
                    print(line, flush=True)
                    self.last_debug_print = time.monotonic()
            elif line.startswith(('ACK ', 'ERR ', 'READY')):
                print(line, flush=True)
            if acknowledged:
                return True
        return acknowledged

    def stopped(self):
        return bool(self.debug and self.debug.get('MODE') == 'IDLE'
                    and self.debug.get('PWM') == '0/0'
                    and self.debug.get('AUTOEN') == '0')

    def stop(self, verify=False):
        self.last_stop_ok = False
        self.send('STOP')
        ack = self.pump(1.0, expected_ack='ACK STOP')
        if verify:
            self.debug = None
            self.pump(0.7)
            if not ack or not self.stopped():
                raise RuntimeError('STOP acknowledgement/IDLE/PWM=0 was not verified')
            self.last_stop_ok = True
        return ack

    def check(self):
        if not self.stop():
            raise RuntimeError('No ACK STOP: refusing all motion commands')
        self.send('DEBUG 1')
        if not self.pump(1.0, expected_ack='ACK DEBUG 1'):
            raise RuntimeError('No ACK DEBUG 1: firmware protocol not confirmed')
        self.debug = None
        self.pump(1.2)
        if not self.stopped():
            raise RuntimeError('Firmware did not report IDLE, PWM=0/0, AUTOEN=0')
        if 'HARDWARE ERROR' in self.debug.get('DOING', ''):
            raise RuntimeError('Firmware reports hardware initialization failure')
        print('PASS: firmware handshake and stopped state.', flush=True)
        for name in ('FRONT', 'LEFT', 'RIGHT'):
            value = self.sonars.get(name)
            print(f'{name}: {value if value is not None else "no US telemetry received"}',
                  flush=True)
        print('VALID=0/noecho is unknown range; this probe does not treat it as clear.',
              flush=True)

    def pulse(self, label, v, w, duration=0.6):
        self.stop(verify=True)
        self.saw_pwm = False
        print(f'TEST {label}: CMD v {v:.3f} w {w:.3f} for {duration:.1f}s', flush=True)
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                self.send(f'CMD v {v:.3f} w {w:.3f}')
                self.pump(min(0.1, max(0, deadline - time.monotonic())))
        finally:
            self.stop(verify=True)
        if not self.saw_pwm:
            raise RuntimeError(f'{label}: no nonzero MANUAL PWM reported; check veto/debug output')
        print(f'PASS: {label} produced nonzero firmware PWM then returned to IDLE. '
              'Physical direction/movement must be observed.', flush=True)

    def watchdog(self):
        self.stop(verify=True)
        self.saw_pwm = False
        print('TEST watchdog: brief forward command, then 1.2s with no commands.', flush=True)
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            self.send('CMD v 0.050 w 0.000')
            self.pump(min(0.1, max(0, deadline - time.monotonic())))
        if not self.saw_pwm:
            raise RuntimeError('No nonzero PWM before watchdog test; cannot verify active-motion timeout')
        self.debug = None
        self.pump(1.2)
        if not self.stopped():
            raise RuntimeError('Watchdog did not report IDLE/PWM=0 after commands ceased')
        print('PASS: command loss returned firmware to IDLE/PWM=0 within the observation window. '
              'This does not measure physical stopping time or distance.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--test', choices=['check', 'wheels'], default='check')
    args = parser.parse_args()
    if not Path(args.port).exists():
        parser.error(f'Serial port does not exist: {args.port}')
    owners = port_users(args.port)
    if owners:
        parser.error(f'Port already open: {owners}; stop those serial clients first')
    try:
        import serial
    except ImportError:
        parser.error('pyserial is required in this Python environment')

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    ser = serial.Serial(port=None, baudrate=115200, timeout=0.05,
                        write_timeout=0.3, exclusive=True)
    # Avoid intentionally toggling USB-UART reset/boot lines on open.
    # Some adapters/drivers can still glitch these lines: start with wheels raised.
    ser.dtr = False
    ser.rts = False
    ser.port = args.port
    probe = Probe(ser)
    code = 0
    try:
        ser.open()
        time.sleep(0.4)
        ser.reset_input_buffer()
        probe.check()
        if args.test == 'wheels':
            probe.pulse('forward: both wheels forward', 0.050, 0.000)
            probe.pulse('left spin: left backward, right forward', 0.000, 0.250)
            probe.pulse('right spin: left forward, right backward', 0.000, -0.250)
            probe.watchdog()
    except KeyboardInterrupt:
        print('Interrupted; requesting STOP.', flush=True)
        code = 130
    except Exception as exc:
        print(f'FAIL: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        code = 1
    finally:
        if ser.is_open:
            try:
                probe.stop(verify=True)
                print('Final STOP acknowledged; firmware reports IDLE/PWM=0.', flush=True)
                probe.send('DEBUG 0')
            except Exception as exc:
                print(f'STOP NOT VERIFIED: {exc}. Use the motor-power cutoff.',
                      file=sys.stderr, flush=True)
                code = 1
            finally:
                ser.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
