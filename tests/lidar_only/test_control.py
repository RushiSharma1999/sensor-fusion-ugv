"""Offline callback tests; ROS, DDS, serial hardware and dynamics are not tested.

Run: python3 -m unittest discover -s tests -v
"""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


def ns(**kwargs):
    return types.SimpleNamespace(**kwargs)


class Twist:
    def __init__(self):
        self.linear = ns(x=0.0, y=0.0, z=0.0)
        self.angular = ns(x=0.0, y=0.0, z=0.0)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class FakeNode:
    overrides = {}
    ros_time = 1000.0

    def __init__(self, name):
        self.params = {}
        self.logs = []

    def declare_parameter(self, name, default):
        self.params[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        value = self.params[name]
        return ns(value=value, get_parameter_value=lambda: ns(
            string_value=value, integer_value=value))

    def create_subscription(self, *args):
        return args

    def create_publisher(self, *args):
        return Publisher()

    def create_timer(self, period, callback):
        return ns(period=period, callback=callback)

    def get_clock(self):
        return ns(now=lambda: ns(nanoseconds=int(self.ros_time * 1e9),
                                 to_msg=lambda: ns(sec=int(self.ros_time), nanosec=0)))

    def get_logger(self):
        return ns(**{level: lambda text, level=level: self.logs.append((level, text))
                     for level in ('info', 'warn', 'error')})


class Serial:
    def __init__(self, *args, **kwargs):
        self.writes = []
        self.error = False
        self.partial = False
        self.data = b''

    def write(self, data):
        self.writes.append(data)
        if self.error:
            raise IOError('unplugged')
        return len(data) - 1 if self.partial else len(data)

    def read(self, size):
        if self.error:
            raise IOError('unplugged')
        value, self.data = self.data, b''
        return value


def install_stubs():
    def module(name, **kwargs):
        value = types.ModuleType(name)
        value.__dict__.update(kwargs)
        sys.modules[name] = value

    module('rclpy')
    module('rclpy.node', Node=FakeNode)
    module('rclpy.qos', qos_profile_sensor_data=object())
    for pkg in ('geometry_msgs', 'sensor_msgs', 'std_msgs', 'nav_msgs'):
        module(pkg)
    module('geometry_msgs.msg', Twist=Twist, TransformStamped=type('TransformStamped', (), {}))
    module('sensor_msgs.msg', LaserScan=type('LaserScan', (), {}))
    module('std_msgs.msg', **{x: type(x, (), {}) for x in ('Bool', 'Float32', 'String', 'Float32MultiArray')})
    module('nav_msgs.msg', Odometry=type('Odometry', (), {}))
    module('tf2_ros', TransformBroadcaster=lambda node: ns(sendTransform=lambda t: None))
    module('serial', Serial=Serial)


install_stubs()


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VFH = load('patched_vfh', 'ws/src/vfh_pkg/vfh_pkg/vfh_node.py')
MUX = load('patched_mux', 'ws/src/vision_uart_pkg/vision_uart_pkg/safety_mux_node.py')
BRIDGE = load('patched_bridge', 'ws/src/esp_bridge/esp_bridge/cmd_vel_to_serial.py')
OLD = load('original_vfh', 'tests/lidar_only/fixtures/original_vfh.py')


def scan(value=3.0, stamp=1000.0):
    return ns(ranges=[value] * 321, angle_min=math.radians(-80),
              angle_max=math.radians(80), angle_increment=math.radians(0.5),
              range_min=0.10, range_max=10.0,
              header=ns(stamp=ns(sec=int(stamp), nanosec=round((stamp % 1) * 1e9))))


def obstacle(msg, low=-10, high=10, distance=0.7):
    for i in range(len(msg.ranges)):
        angle = math.degrees(msg.angle_min + i * msg.angle_increment)
        if low <= angle <= high:
            msg.ranges[i] = distance
    return msg


def command(v=0.08, w=0.0):
    msg = Twist()
    msg.linear.x, msg.angular.z = v, w
    return msg


class ControlTests(unittest.TestCase):
    def setUp(self):
        FakeNode.overrides = {'forward_speed': 0.08, 'min_speed': 0.03,
                              'max_yaw_rate': 0.4, 'front_stop_dist': 0.5}
        FakeNode.ros_time = 1000.0
        self.monotonic = 10.0
        self.time_patch = patch('time.monotonic', lambda: self.monotonic)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)

    def advance(self, seconds=0.084):
        self.monotonic += seconds
        FakeNode.ros_time += seconds

    def latest(self, node):
        return node.cmd_pub.messages[-1]

    def assertZero(self, msg):
        self.assertEqual((msg.linear.x, msg.linear.y, msg.linear.z,
                          msg.angular.x, msg.angular.y, msg.angular.z), (0,) * 6)

    def test_original_all_inf_requests_forward(self):
        node = OLD.VFHObstacleAvoidance()
        node.scan_callback(scan(float('inf')))
        self.assertGreater(self.latest(node).linear.x, 0)

    def test_original_critical_stop_requests_rotation(self):
        node = OLD.VFHObstacleAvoidance()
        node.on_fusion_stop(ns(data=True))
        node.scan_callback(scan())
        self.assertNotEqual(self.latest(node).angular.z, 0)

    def test_original_speed_floor_overrides_tiny_limit(self):
        node = OLD.VFHObstacleAvoidance()
        node.on_fusion_speed(ns(data=0.01))
        node.scan_callback(scan())
        self.assertGreaterEqual(self.latest(node).linear.x, 0.03)

    def test_new_startup_stops(self):
        node = VFH.VFHObstacleAvoidance()
        node.check_scan_timeout()
        self.assertZero(self.latest(node))

    def test_clear_finite_scan_allows_bounded_forward(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(scan())
        self.assertAlmostEqual(self.latest(node).linear.x, 0.08)
        self.assertAlmostEqual(self.latest(node).angular.z, 0)

    def test_invalid_scan_variants_stop(self):
        for value in (float('inf'), float('-inf'), float('nan'), 0.0, -1.0, 0.05, 11.0):
            with self.subTest(value=value):
                node = VFH.VFHObstacleAvoidance()
                node.scan_callback(scan(value))
                self.assertZero(self.latest(node))

    def test_empty_after_motion_stops_immediately(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(scan())
        msg = scan(stamp=1000.084)
        msg.ranges = []
        node.scan_callback(msg)
        self.assertZero(self.latest(node))

    def test_bad_metadata_variants_rejected_in_both_guards(self):
        for field, value in [('angle_increment', 0.0), ('angle_max', float('nan')),
                             ('range_min', -1), ('range_max', 0.05),
                             ('angle_max', 1.0), ('angle_min', 0.0)]:
            msg = scan()
            setattr(msg, field, value)
            with self.subTest(field=field, value=value):
                for mod in (VFH, MUX):
                    self.assertFalse(mod.inspect_scan(msg, 1000.0, 0.5, math.radians(80))[0])

    def test_stale_and_future_and_zero_stamp_rejected(self):
        for stamp in (0, 999.0, 1000.2):
            node = VFH.VFHObstacleAvoidance()
            node.scan_callback(scan(stamp=stamp))
            self.assertZero(self.latest(node))

    def test_replayed_stamp_stops(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(scan())
        node.scan_callback(scan())
        self.assertZero(self.latest(node))

    def test_no_front_return_stops_despite_side_returns(self):
        msg = obstacle(scan(), -16, 16, float('inf'))
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(msg)
        self.assertZero(self.latest(node))

    def test_unknown_sectors_are_not_open(self):
        node = VFH.VFHObstacleAvoidance()
        msg = scan(float('inf'))
        msg.ranges[160] = 3.0
        node.scan_callback(msg)
        self.assertZero(self.latest(node))  # one observed sector is no passage

    def test_left_obstacle_prefers_right(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(obstacle(scan(), 10, 50, 0.7))
        self.assertLess(self.latest(node).angular.z, 0)

    def test_right_obstacle_prefers_left(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(obstacle(scan(), -50, -10, 0.7))
        self.assertGreater(self.latest(node).angular.z, 0)

    def test_front_stop_zeroes_both_components(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(obstacle(scan(), distance=0.49))
        self.assertZero(self.latest(node))

    def test_no_valley_stops_instead_of_escape(self):
        node = VFH.VFHObstacleAvoidance()
        for i in range(10):
            self.advance()
            node.scan_callback(scan(0.6, FakeNode.ros_time))
            self.assertZero(self.latest(node))

    def test_small_positive_speed_not_raised_to_floor(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(obstacle(scan(), distance=0.501))
        self.assertGreater(self.latest(node).linear.x, 0)
        self.assertLess(self.latest(node).linear.x, 0.03)

    def test_history_preserves_hazard_after_single_clear_frame(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(obstacle(scan(), distance=0.7))
        self.advance()
        node.scan_callback(scan(stamp=FakeNode.ros_time))
        self.assertNotAlmostEqual(self.latest(node).angular.z, 0)
        self.assertLess(self.latest(node).linear.x, 0.08)
        self.advance(0.6)
        node.scan_callback(scan(stamp=FakeNode.ros_time))
        # History expired, but command EMA may still retain some yaw.
        self.assertAlmostEqual(node.prev_target_angle, 0)

    def test_history_never_makes_all_inf_usable(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(scan())
        self.advance()
        node.scan_callback(scan(float('inf'), FakeNode.ros_time))
        self.assertZero(self.latest(node))

    def test_vfh_watchdog_stops_and_clears_history(self):
        node = VFH.VFHObstacleAvoidance()
        node.scan_callback(scan())
        self.advance(0.251)
        node.check_scan_timeout()
        self.assertZero(self.latest(node))
        self.assertFalse(node.scan_history)

    def test_fusion_disabled_in_lidar_only_vfh(self):
        node = VFH.VFHObstacleAvoidance()
        node.on_fusion_stop(ns(data=True))
        node.on_fusion_speed(ns(data=0.0))
        node.scan_callback(scan())
        self.assertGreater(self.latest(node).linear.x, 0)

    def test_enabled_fusion_stop_is_zero_not_escape(self):
        FakeNode.overrides['use_fusion'] = True
        node = VFH.VFHObstacleAvoidance()
        node.on_fusion_stop(ns(data=True))
        node.scan_callback(scan())
        self.assertZero(self.latest(node))

    def test_nonfinite_fusion_speed_zero_when_enabled(self):
        FakeNode.overrides['use_fusion'] = True
        node = VFH.VFHObstacleAvoidance()
        node.on_fusion_speed(ns(data=float('nan')))
        node.scan_callback(scan())
        self.assertZero(self.latest(node))

    def test_mux_startup_zero_even_with_candidate(self):
        node = MUX.SafetyMuxNode()
        node.on_vfh_cmd(command())
        node.control_loop()
        self.assertZero(self.latest(node))

    def test_mux_allows_fresh_scan_and_vfh(self):
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        node.on_vfh_cmd(command())
        node.control_loop()
        self.assertAlmostEqual(self.latest(node).linear.x, 0.08)

    def test_mux_scan_loss_stops_despite_fresh_vfh(self):
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        self.advance(0.251)
        node.on_vfh_cmd(command())
        node.control_loop()
        self.assertZero(self.latest(node))

    def test_mux_candidate_loss_stops_despite_fresh_scan(self):
        node = MUX.SafetyMuxNode()
        node.on_vfh_cmd(command())
        self.advance(0.501)
        node.on_scan(scan(stamp=FakeNode.ros_time))
        node.control_loop()
        self.assertZero(self.latest(node))

    def test_mux_invalid_scan_stops_immediately(self):
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        node.on_vfh_cmd(command())
        node.control_loop()
        self.advance()
        node.on_scan(scan(float('inf'), FakeNode.ros_time))
        self.assertZero(self.latest(node))

    def test_mux_near_front_stops_both_axes(self):
        node = MUX.SafetyMuxNode()
        node.on_vfh_cmd(command(0.08, 0.3))
        node.on_scan(obstacle(scan(), distance=0.49))
        node.control_loop()
        self.assertZero(self.latest(node))

    def test_mux_nonfinite_candidate_zero(self):
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        for v, w in [(float('nan'), 0), (0.08, float('inf'))]:
            node.on_vfh_cmd(command(v, w))
            node.control_loop()
            self.assertZero(self.latest(node))

    def test_mux_limits_preserve_curvature(self):
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        node.on_vfh_cmd(command(0.16, 0.8))
        node.control_loop()
        self.assertAlmostEqual(self.latest(node).linear.x, 0.08)
        self.assertAlmostEqual(self.latest(node).angular.z, 0.4)

    def test_mux_lidar_only_ignores_camera_and_optional_fusion(self):
        FakeNode.overrides['prefer_camera_when_target_seen'] = True
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        node.on_camera_cmd(command(0.01, -0.3))
        node.on_target_seen(ns(data=True))
        node.on_critical_stop(ns(data=True))
        node.on_speed_limit(ns(data=0.0))
        node.on_vfh_cmd(command())
        node.control_loop()
        self.assertAlmostEqual(self.latest(node).linear.x, 0.08)

    def test_mux_optional_fusion_requires_fresh_messages(self):
        FakeNode.overrides['lidar_only_mode'] = False
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        node.on_vfh_cmd(command())
        node.control_loop()
        self.assertZero(self.latest(node))

    def test_mux_optional_fusion_zero_speed_stops_rotation_too(self):
        FakeNode.overrides['lidar_only_mode'] = False
        node = MUX.SafetyMuxNode()
        node.on_scan(scan())
        node.on_vfh_cmd(command(0.08, 0.3))
        node.on_critical_stop(ns(data=False))
        node.on_speed_limit(ns(data=0.0))
        node.control_loop()
        self.assertZero(self.latest(node))

    def test_bridge_stale_cmd_sends_zero(self):
        node = BRIDGE.CmdVelToSerial()
        node.on_cmd_vel(command())
        self.advance(0.251)
        node.on_cmd_timer()
        self.assertEqual(node.ser.writes[-1], b'CMD v 0.000 w 0.000\n')

    def test_bridge_nonfinite_cmd_sends_zero(self):
        node = BRIDGE.CmdVelToSerial()
        node.on_cmd_vel(command(float('nan'), 0.2))
        node.on_cmd_timer()
        self.assertEqual(node.ser.writes[-1], b'CMD v 0.000 w 0.000\n')

    def test_bridge_caps_commands(self):
        node = BRIDGE.CmdVelToSerial()
        node.on_cmd_vel(command(0.16, 0.8))
        node.on_cmd_timer()
        self.assertEqual(node.ser.writes[-1], b'CMD v 0.080 w 0.400\n')

    def test_bridge_write_fault_latches_zero_after_repair(self):
        node = BRIDGE.CmdVelToSerial()
        node.on_cmd_vel(command())
        node.ser.error = True
        node.on_cmd_timer()
        self.assertTrue(node.serial_fault)
        node.ser.error = False
        node.on_cmd_vel(command())
        node.on_cmd_timer()
        self.assertEqual(node.ser.writes[-1], b'CMD v 0.000 w 0.000\n')

    def test_bridge_partial_write_latches_fault(self):
        node = BRIDGE.CmdVelToSerial()
        node.ser.partial = True
        node.on_cmd_timer()
        self.assertTrue(node.serial_fault)

    def test_bridge_read_fault_latches_fault(self):
        node = BRIDGE.CmdVelToSerial()
        node.ser.error = True
        node.on_rx_timer()
        self.assertTrue(node.serial_fault)

    def test_bridge_nonfinite_odom_discarded(self):
        node = BRIDGE.CmdVelToSerial()
        node.handle_odom_line('ODOM x nan y 0 th 0 v 0 w 0')
        self.assertFalse(node.odom_pub.messages)

    def test_bridge_oversized_rx_discarded(self):
        node = BRIDGE.CmdVelToSerial()
        node._rx_buf = 'x' * 17000
        node.ser.data = b'x'
        node.on_rx_timer()
        self.assertEqual(node._rx_buf, '')

    def test_v4_us_telemetry_dispatched_and_aggregated(self):
        node = BRIDGE.CmdVelToSerial()
        node.ser.data = (b'US|ID:FRONT|T:100|VALID:1|CM:25.0\n'
                         b'US|ID:LEFT|T:110|VALID:1|CM:70.0\n'
                         b'US|ID:RIGHT|T:120|VALID:0|CM:400.0\n')
        node.on_rx_timer()
        fields = dict(p.split(':') for p in node.ultra_raw_pub.messages[-1].data.split('|'))
        self.assertEqual(fields['FRONT'], '25.0')
        self.assertEqual(fields['LEFT'], '70.0')
        self.assertEqual(fields['VALID_FRONT'], '1')
        self.assertEqual(fields['VALID_RIGHT'], '0')
        self.assertEqual(fields['VALID_BACK'], '0')
        self.assertFalse(node.odom_pub.messages)

    def test_v4_repeated_sample_does_not_extend_freshness(self):
        node = BRIDGE.CmdVelToSerial()
        line = 'US|ID:FRONT|T:123|VALID:1|CM:50.0'
        node.handle_v4_us_line(line)
        self.advance(.6)
        node.handle_v4_us_line(line)
        self.advance(.21)
        node.on_rx_timer()  # Also expires data when serial is silent.
        self.assertIn('VALID_FRONT:0', node.ultra_raw_pub.messages[-1].data)
        node.handle_v4_us_line('US|ID:FRONT|T:124|VALID:1|CM:51.0')
        self.assertIn('VALID_FRONT:1', node.ultra_raw_pub.messages[-1].data)

    def test_v4_invalid_sample_overrides_previous_valid(self):
        node = BRIDGE.CmdVelToSerial()
        node.handle_v4_us_line('US|ID:FRONT|T:123|VALID:1|CM:50.0')
        node.handle_v4_us_line('US|ID:FRONT|T:123|VALID:0|CM:400.0')
        self.assertIn('VALID_FRONT:0', node.ultra_raw_pub.messages[-1].data)

    def test_v4_bad_us_messages_never_become_clear(self):
        node = BRIDGE.CmdVelToSerial()
        for line in ('US|ID:FRONT|T:1|VALID:1|CM:nan',
                     'US|ID:FRONT|T:1|VALID:1|CM:inf',
                     'US|ID:UNKNOWN|T:1|VALID:1|CM:20',
                     'US|ID:FRONT|T:1|VALID:2|CM:20',
                     'US|ID:FRONT|T:1|VALID:1|CM:0',
                     'US|ID:FRONT|T:-1|VALID:1|CM:20',
                     'US|ID:FRONT|T:1|VALID:1|CM:20|CM:400', 'US|garbage'):
            node.handle_v4_us_line(line)
        self.assertFalse(node.ultra_raw_pub.messages)

    def test_v4_timestamp_wrap_can_accept_new_measurement(self):
        node = BRIDGE.CmdVelToSerial()
        node.handle_v4_us_line('US|ID:FRONT|T:4294967295|VALID:1|CM:80')
        self.advance(.1)
        node.handle_v4_us_line('US|ID:FRONT|T:2|VALID:1|CM:81')
        self.assertTrue(node.ultra_raw_pub.messages[-1].data.startswith('T:2|'))

    def test_v4_debug_published_without_changing_lidar_command(self):
        node = BRIDGE.CmdVelToSerial()
        node.on_cmd_vel(command(.04,.2))
        debug = 'DBG|T:200|MODE:MANUAL|DOING:BLOCKED BY VETO|WHY:front obstacle|PWM:0/0|AUTOEN:0|ENC:0/0'
        node.ser.data = (debug + '\n').encode()
        node.on_rx_timer()
        node.on_cmd_timer()
        self.assertEqual(node.motor_status_pub.messages[-1].data, debug)
        self.assertEqual(node.ser.writes[-1], b'CMD v 0.040 w 0.200\n')

    def test_debug_enable_survives_serial_open_reset_without_extra_motion(self):
        FakeNode.overrides['debug_serial'] = True
        node = BRIDGE.CmdVelToSerial()
        self.advance(1.1)
        node.on_cmd_timer()
        self.assertEqual(node.ser.writes, [b'DEBUG 1\n', b'CMD v 0.000 w 0.000\n'])
        node.on_cmd_timer()
        self.assertEqual(node.ser.writes.count(b'DEBUG 1\n'), 1)

    def test_aggregate_ultra_telemetry_still_supported(self):
        node = BRIDGE.CmdVelToSerial()
        node.ser.data = b'ULTRA|T:5|FRONT:33.0|LEFT:400.0|RIGHT:400.0|BACK:400.0|VALID_FRONT:1|VALID_LEFT:0|VALID_RIGHT:0|VALID_BACK:0\n'
        node.on_rx_timer()
        self.assertIn('FRONT:33.0', node.ultra_raw_pub.messages[-1].data)

    def test_invalid_config_rejected(self):
        for key, value in [('ema_alpha', 2.0), ('max_yaw_rate', 0.0),
                           ('front_slowdown_dist', 0.1), ('obstacle_hold_sec', 2.0)]:
            with self.subTest(key=key):
                previous = FakeNode.overrides.copy()
                FakeNode.overrides[key] = value
                with self.assertRaises(ValueError):
                    VFH.VFHObstacleAvoidance()
                FakeNode.overrides = previous


if __name__ == '__main__':
    unittest.main()
