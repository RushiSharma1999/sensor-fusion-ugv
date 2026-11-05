"""
Exercise the real bridge with a fake serial port, ROS API and monotonic clock.

These are protocol/callback regressions, not firmware or ROS middleware tests.
No serial device is opened, and no motor command leaves this process.
"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from esp_bridge.speed_profiles import speed_limits


class Twist:
    def __init__(self):
        self.linear = NS(x=0.0, y=0.0, z=0.0)
        self.angular = NS(x=0.0, y=0.0, z=0.0)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.params = {}
        self.messages = []
        self.logs = []

    def declare_parameter(self, name, default):
        self.params[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        value = self.params[name]
        return NS(
            value=value,
            get_parameter_value=lambda: NS(
                string_value=value,
                integer_value=value))

    def get_logger(self):
        return NS(info=self.logs.append, warn=self.logs.append, error=self.logs.append)

    def create_publisher(self, *args):
        return NS(publish=self.messages.append)

    def create_subscription(self, *args):
        return NS()

    def create_timer(self, *args):
        return NS(cancel=lambda: None)


class SerialPort:
    def __init__(self, **kwargs):
        self.settings = kwargs
        self.dtr = self.rts = True
        self.opened = False
        self.closed = False
        self.incoming = b''
        self.written = []
        self.fail_write = self.fail_read = self.short_write = False
        self.ack_on_stop = False

    def open(self):  # noqa: A003 - pyserial interface
        assert self.dtr is False and self.rts is False
        assert self.settings['exclusive'] is True
        assert self.settings['port'] is None
        self.opened = True

    def write(self, data):
        if self.fail_write:
            raise OSError('synthetic write fault')
        self.written.append(data.decode('ascii'))
        if self.ack_on_stop and data == b'STOP\n':
            self.incoming += b'ACK STOP\n'
        return len(data) - 1 if self.short_write else len(data)

    def read(self, n):
        if self.fail_read:
            raise OSError('synthetic read fault')
        data, self.incoming = self.incoming[:n], self.incoming[n:]
        return data

    def close(self):
        self.closed = True


def module(name, **fields):
    result = ModuleType(name)
    result.__dict__.update(fields)
    return result


def load_bridge():
    """Isolate stubs so actual ROS integration imports remain unaffected."""
    stubs = {
        'rclpy': module('rclpy'),
        'rclpy.node': module('rclpy.node', Node=FakeNode),
        'rclpy.executors': module('rclpy.executors', ExternalShutdownException=RuntimeError),
        'geometry_msgs': module('geometry_msgs'),
        'geometry_msgs.msg': module('geometry_msgs.msg', Twist=Twist, TransformStamped=NS),
        'nav_msgs': module('nav_msgs'),
        'nav_msgs.msg': module('nav_msgs.msg', Odometry=NS),
        'std_msgs': module('std_msgs'),
        'std_msgs.msg': module('std_msgs.msg', String=NS),
        'tf2_ros': module('tf2_ros', TransformBroadcaster=lambda node: NS()),
        'serial': module('serial', Serial=SerialPort),
    }
    path = Path(__file__).parents[1] / 'esp_bridge' / 'cmd_vel_to_serial.py'
    spec = importlib.util.spec_from_file_location('esp_bridge._offline_serial_test', path)
    bridge = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(bridge)
    return bridge


BRIDGE = load_bridge()


class SerialSafetyTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.clock_patch = patch.object(BRIDGE, 'time', NS(
            monotonic=lambda: self.now, sleep=self.advance))
        self.clock_patch.start()
        FakeNode.overrides = {}
        self.node = None

    def tearDown(self):
        self.clock_patch.stop()
        FakeNode.overrides = {}

    def advance(self, seconds):
        self.now += seconds

    def make(self, **overrides):
        FakeNode.overrides = overrides
        self.node = BRIDGE.CmdVelToSerial()
        return self.node

    def ack(self):
        self.node.ser.incoming += b'ACK STOP\n'
        self.node.on_rx_timer()

    def command(self, v=.05, w=.25):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.node.on_cmd_vel(msg)
        self.node.on_cmd_timer()

    def test_dtr_rts_exclusive_startup_ack_and_pre_ack_command_discard(self):
        node = self.make(dry_run=False)
        self.assertTrue(node.ser.opened)
        self.command()
        self.assertEqual(node.ser.written, ['STOP\n'])
        self.ack()
        node.on_cmd_timer()
        self.assertIn('CMD v 0.000 w 0.000\n', node.ser.written)
        self.command()
        self.assertEqual(node.ser.written[-1], 'CMD v 0.050 w 0.250\n')

    def test_dry_run_only_zero_commands_including_turns(self):
        node = self.make()
        self.ack()
        self.command(.08, -.25)
        self.assertEqual(node.ser.written[-1], 'CMD v 0.000 w 0.000\n')

    def test_missing_ack_latches_fault_and_late_ack_cannot_arm(self):
        node = self.make(dry_run=False)
        self.advance(5.1)
        node.on_cmd_timer()
        self.ack()
        self.command()
        self.assertTrue(node.serial_fault)
        self.assertFalse(node.stop_acknowledged)
        self.assertFalse(any(x.startswith('CMD') for x in node.ser.written))

    def test_firmware_reboot_latches_after_arming(self):
        node = self.make(dry_run=False)
        self.ack()
        self.command()
        node.ser.incoming = b'READY ESP32\n'
        node.on_rx_timer()
        self.ack()
        before = len(node.ser.written)
        self.command()
        self.assertTrue(node.serial_fault)
        self.assertFalse(any(x.startswith('CMD') for x in node.ser.written[before:]))

    def test_ready_before_handshake_remains_stopped(self):
        node = self.make(dry_run=False)
        node.ser.incoming = b'READY ESP32\n'
        node.on_rx_timer()
        self.assertFalse(node.serial_fault)
        self.assertFalse(node.stop_acknowledged)
        self.ack()
        self.command()
        self.assertEqual(node.ser.written[-1], 'CMD v 0.050 w 0.250\n')

    def test_serial_read_write_and_short_write_faults_disable_motion(self):
        for fault in ('fail_read', 'fail_write', 'short_write'):
            node = self.make(dry_run=False)
            self.ack()
            setattr(node.ser, fault, True)
            if fault == 'fail_read':
                node.on_rx_timer()
            else:
                self.command()
            self.assertTrue(node.serial_fault, fault)
            setattr(node.ser, fault, False)
            before = len(node.ser.written)
            self.advance(.3)
            self.command()
            self.assertTrue(all(x == 'STOP\n' for x in node.ser.written[before:]))

    def test_bridge_expiry_is_independent_and_nonfinite_stops(self):
        node = self.make(dry_run=False)
        self.ack()
        self.command()
        self.advance(.251)
        node.on_cmd_timer()
        self.assertEqual(node.ser.written[-1], 'CMD v 0.000 w 0.000\n')
        for value in (float('nan'), float('inf')):
            self.command(value, .25)
            self.assertEqual(node.ser.written[-1], 'CMD v 0.000 w 0.000\n')

    def test_both_profiles_match_shared_limits_and_reverse_disabled(self):
        for profile in ('conservative', 'open_space'):
            node = self.make(dry_run=False, speed_profile=profile)
            self.ack()
            self.command(1, 1)
            limits = speed_limits(profile)
            self.assertEqual(node.max_linear_speed, limits.linear)
            self.assertEqual(node.max_angular_speed, limits.angular)
            self.assertEqual(node.ser.written[-1], f'CMD v {limits.linear:.3f} w 0.250\n')
            self.command(-.05, -1)
            self.assertEqual(node.ser.written[-1], 'CMD v 0.000 w -0.250\n')
        with self.assertRaises(ValueError):
            self.make(max_linear_speed=.08)
        with self.assertRaises(ValueError):
            self.make(max_linear_speed=.03)

    def test_shutdown_stop_ack_and_fault_reporting(self):
        for fault in (False, True):
            node = self.make(dry_run=False)
            self.ack()
            self.command()
            node.ser.ack_on_stop = True
            node.ser.fail_read = fault
            node.close()
            self.assertTrue(node.ser.closed)
            self.assertEqual(node.ser.written[-1], 'STOP\n')
            expected = f'Final STOP acknowledgement observed={int(not fault)}'
            self.assertTrue(any(expected in entry for entry in node.logs))

    def test_shutdown_reports_absent_ack(self):
        node = self.make()
        self.ack()
        node.close()
        self.assertTrue(node.ser.closed)
        self.assertIn('Final STOP acknowledgement observed=0', node.logs)

    def test_shutdown_close_fault_still_reports_ack(self):
        node = self.make()
        self.ack()
        node.ser.ack_on_stop = True
        with patch.object(node.ser, 'close', side_effect=OSError('synthetic close failure')):
            node.close()
        self.assertTrue(node.serial_fault)
        self.assertIn('Final STOP acknowledgement observed=1', node.logs)

    def test_serial_buffer_overflow_latches(self):
        node = self.make(dry_run=False)
        self.ack()
        node.ser.incoming = b'x' * 17000
        for _ in range(5):
            node.on_rx_timer()
        self.assertTrue(node.serial_fault)

    def test_ultrasonic_telemetry_format_and_expiry(self):
        node = self.make()
        node.ser.incoming = b'US|ID:FRONT|CM:42.0|VALID:1|T:123\n'
        node.on_rx_timer()
        node.publish_ultra_samples(self.now)
        self.assertIn('T:123|FRONT:42.0', node.messages[-1].data)
        self.assertIn('VALID_FRONT:1', node.messages[-1].data)
        self.advance(.7)
        node.publish_ultra_samples(self.now)
        self.assertIn('FRONT:400.0', node.messages[-1].data)
        self.assertIn('VALID_FRONT:0', node.messages[-1].data)


if __name__ == '__main__':
    unittest.main()
