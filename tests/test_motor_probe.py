"""Exercise bounded wheels-raised curve commissioning with fake serial and time."""

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'motor_probe_tested', Path(__file__).resolve().parents[1] / 'ugv_motor_check.py')
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


class FakePort:
    def __init__(self, clock, fault=None, **settings):
        self.clock, self.fault, self.settings = clock, fault, settings
        self.dtr = self.rts = True
        self.is_open = False
        self.queue, self.writes = [], []
        self.active = False
        self.last_command = -100

    def open(self):  # noqa: A003 - serial interface
        assert self.dtr is False and self.rts is False
        assert self.settings['exclusive'] is True
        assert self.settings['port'] is None
        self.is_open = True

    def close(self):
        self.is_open = False

    def reset_input_buffer(self):
        self.queue.clear()

    def flush(self):
        pass

    def write(self, data):
        line = data.decode().strip()
        self.writes.append((self.clock.now, line))
        if line == 'STOP':
            self.active = False
            if self.fault != 'missing_ack':
                self.queue.append(b'ACK STOP\n')
        elif line == 'DEBUG 1':
            self.queue.append(b'ACK DEBUG 1\n')
        elif line.startswith('CMD '):
            self.active = True
            self.last_command = self.clock.now
            if self.fault == 'reboot':
                self.queue.append(b'READY ESP32\n')
            if self.fault == 'firmware_error':
                self.queue.append(b'ERR synthetic firmware fault\n')
            if self.fault == 'write':
                raise OSError('synthetic serial write fault')
            if self.fault == 'short':
                return len(data) - 1
        return len(data)

    def readline(self, size):
        self.clock.now += .04
        if self.fault == 'read' and self.active:
            raise OSError('synthetic serial read fault')
        if self.queue:
            return self.queue.pop(0)
        if self.clock.now - self.last_command > .5:
            self.active = False
        mode, pwm = ('MANUAL', '64/70') if self.active else ('IDLE', '0/0')
        return f'DBG|MODE:{mode}|PWM:{pwm}|AUTOEN:0\n'.encode()


class MotorProbeTests(unittest.TestCase):
    def run_probe(self, args=(), fault=None):
        clock = NS(now=0.0)
        clock.monotonic = lambda: clock.now
        clock.sleep = lambda delay: setattr(clock, 'now', clock.now + delay)
        created = []

        def serial_factory(**settings):
            port = FakePort(clock, fault, **settings)
            created.append(port)
            return port

        with (
            patch.object(PROBE, 'time', clock),
            patch.object(PROBE, 'port_users', return_value=[]),
            patch.object(PROBE.Path, 'exists', return_value=True),
            patch.object(PROBE.signal, 'signal'),
            patch.dict(sys.modules, {'serial': NS(Serial=serial_factory)}),
            patch.object(sys, 'argv', ['ugv_motor_check.py', *args]),
            redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO())
        ):
            code = PROBE.main()
        return code, created

    def test_default_check_is_stopped_and_preserves_line_controls(self):
        code, ports = self.run_probe()
        self.assertEqual(code, 0)
        self.assertFalse(ports[0].is_open)
        self.assertFalse(any(line.startswith('CMD ') for _, line in ports[0].writes))

    def test_curve_mode_requires_both_explicit_motor_and_raised_flags(self):
        for args in (['--test', 'wheels-curves'],
                     ['--test', 'wheels-curves', '--motors'],
                     ['--test', 'wheels-curves', '--wheels-raised']):
            with self.assertRaises(SystemExit) as error:
                self.run_probe(args)
            self.assertEqual(error.exception.code, 2)

    def test_curve_pulses_are_bounded_signed_and_stopped(self):
        code, ports = self.run_probe(['--test', 'wheels-curves', '--motors', '--wheels-raised',
                                     '--angular-command-sign', '-1'])
        self.assertEqual(code, 0)
        phases, phase = [], []
        for stamp, line in ports[0].writes:
            if line.startswith('CMD '):
                self.assertEqual(line.split()[2], '0.020')
                phase.append((stamp, float(line.split()[4])))
            elif line == 'STOP' and phase:
                phases.append(phase)
                self.assertLessEqual(stamp - phase[0][0], .7)
                phase = []
        self.assertEqual(len(phases), 2)
        self.assertTrue(all(w == -.06 for _, w in phases[0]))
        self.assertTrue(all(w == .06 for _, w in phases[1]))
        self.assertFalse(ports[0].is_open)

    def test_missing_ack_and_serial_reboot_faults_end_motion(self):
        for fault in ('missing_ack', 'reboot', 'firmware_error', 'read', 'write', 'short'):
            code, ports = self.run_probe(
                ['--test', 'wheels-curves', '--motors', '--wheels-raised'], fault)
            self.assertEqual(code, 1, fault)
            commands = [line for _, line in ports[0].writes if line.startswith('CMD ')]
            self.assertLessEqual(len(commands), 1, fault)
            if fault == 'missing_ack':
                self.assertFalse(commands)
            if commands:
                final_cmd = max(i for i, (_, line) in enumerate(ports[0].writes)
                                if line.startswith('CMD '))
                self.assertTrue(any(line == 'STOP' for _, line in ports[0].writes[final_cmd + 1:]))
            self.assertFalse(ports[0].is_open)

    def test_existing_wheels_sequence_and_watchdog_remain_available(self):
        code, ports = self.run_probe(['--test', 'wheels', '--angular-command-sign', '-1'])
        self.assertEqual(code, 0)
        commands = [line for _, line in ports[0].writes if line.startswith('CMD ')]
        self.assertIn('CMD v 0.000 w -0.250', commands)
        self.assertIn('CMD v 0.000 w 0.250', commands)


if __name__ == '__main__':
    unittest.main()
