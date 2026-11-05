"""
Actual ROS timers/topics with synthetic scans and a fake serial transport.

Run after sourcing ROS and this workspace. No LiDAR driver, device, or motor is
accessed. These checks validate the adapter and bridge middleware, not a ride.
"""

import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

try:
    from esp_bridge.cmd_vel_to_serial import CmdVelToSerial
    from geometry_msgs.msg import TransformStamped, Twist
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from vfh_pkg.vfh_node import VFHObstacleAvoidance
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, 'ROS 2 and built packages required')
class RosIntegrationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.probe = Node('synthetic_motion_probe')
        self.executor.add_node(self.probe)
        self.nodes = [self.probe]
        self.received = []

    def tearDown(self):
        for node in reversed(self.nodes):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def spin(self, seconds, action=None):
        deadline = time.monotonic() + seconds
        next_action = 0
        while time.monotonic() < deadline:
            if action and time.monotonic() >= next_action:
                action()
                next_action = time.monotonic() + .12
            self.executor.spin_once(timeout_sec=.005)

    def controller(self, behavior='enhanced', extra=()):
        params = [
            Parameter(
                'behavior',
                value=behavior),
            Parameter(
                'scan_topic',
                value='/test_motion_scan'),
            Parameter(
                'cmd_topic',
                value='/test_motion_cmd')] + list(extra)
        if behavior == 'baseline':
            params += [
                Parameter(
                    'min_speed', value=.02), Parameter(
                    'front_stop_dist', value=.65), Parameter(
                    'hard_turn_dist', value=.85), Parameter(
                    'front_slowdown_dist', value=1.2)]
        node = VFHObstacleAvoidance(parameter_overrides=params)
        self.nodes.append(node)
        self.executor.add_node(node)
        self.sub = self.probe.create_subscription(
            Twist, '/test_motion_cmd', lambda m: self.received.append((time.monotonic(), m)), 10)
        self.pub = self.probe.create_publisher(
            LaserScan, '/test_motion_scan', qos_profile_sensor_data)
        return node

    def scan(self, distance=4.0):
        msg = LaserScan()
        msg.header.stamp = self.probe.get_clock().now().to_msg()
        msg.header.frame_id = 'unilidar_lidar'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180
        msg.range_min = .15
        msg.range_max = 20.
        msg.ranges = [distance] * 361
        return msg

    def test_fixed_timer_stops_after_lidar_ceases_and_duplicates_do_not_refresh(self):
        node = self.controller()
        latest = []

        def publish():
            msg = self.scan()
            latest[:] = [msg]
            self.pub.publish(msg)
        self.spin(1.5, publish)
        self.assertTrue(any(m.linear.x > 0 for _, m in self.received))
        self.received.clear()
        self.spin(.8, lambda: self.pub.publish(latest[0]))
        last = self.received[-1][1]
        self.assertEqual((last.linear.x, last.angular.z), (0, 0))
        self.assertGreater(len(self.received), 10)
        self.assertGreater(node.enhanced.core.last_publication, node.enhanced.core.last_accepted)
        self.assertTrue(all(m.linear.x == 0 and m.angular.z == 0 for t,
                        m in self.received if t > time.monotonic() - .2))

    def test_new_hazard_and_all_infinite_scans_stop_without_ramp(self):
        node = self.controller()
        self.spin(1.2, lambda: self.pub.publish(self.scan()))
        self.assertGreater(self.received[-1][1].linear.x, 0)
        self.received.clear()
        offered = time.monotonic()
        self.spin(.2, lambda: self.pub.publish(self.scan(.25)))
        self.assertTrue(self.received)
        # DDS can still deliver a command published before the new scan callback.
        self.assertTrue(all(m.linear.x == 0 and m.angular.z == 0 for t,
                        m in self.received if t > offered + .08))
        self.assertEqual(node.enhanced.core.previous.v, 0)
        self.spin(.2, lambda: self.pub.publish(self.scan(math.inf)))
        self.assertEqual(self.received[-1][1].linear.x, 0)

    def test_preserved_baseline_still_publishes_and_has_profile_limits(self):
        node = self.controller('baseline')
        self.spin(1.4, lambda: self.pub.publish(self.scan()))
        self.assertTrue(any(m.linear.x > 0 for _, m in self.received))
        self.assertEqual(node.forward_speed, .05)
        self.assertEqual(node.max_yaw_rate, .25)

    def test_transform_missing_stale_and_configured_offset_not_combined(self):
        node = self.controller(
            extra=[
                Parameter(
                    'geometry_mode',
                    value='polygon'),
                Parameter(
                    'transform_mode',
                    value='tf'),
                Parameter(
                    'footprint_xy',
                    value='[[-0.25,-0.18],[0.25,-0.18],[0.25,0.18],[-0.25,0.18]]'),
                Parameter(
                    'lidar_x',
                    value=9.0)])
        self.spin(.5, lambda: self.pub.publish(self.scan()))
        self.assertTrue(self.received)
        self.assertTrue(all(m.linear.x == 0 and m.angular.z == 0 for _, m in self.received))
        msg = self.scan()
        tf = TransformStamped()
        tf.header.stamp.sec = msg.header.stamp.sec
        tf.header.stamp.nanosec = msg.header.stamp.nanosec
        tf.transform.translation.x = .1
        tf.transform.translation.y = .02
        tf.transform.rotation.w = 1.
        fake = type('Buffer', (), {'lookup_transform': lambda *args: tf})()
        node.enhanced.tf_buffer = fake
        mount = node.enhanced.mount_for(msg)
        self.assertEqual(mount.x, .1)
        tf.header.stamp.sec -= 1
        with self.assertRaises(ValueError):
            node.enhanced.mount_for(msg)

    def test_planning_delay_cannot_publish_expired_perception(self):
        node = self.controller()
        self.spin(1.2, lambda: self.pub.publish(self.scan()))
        core = node.enhanced.core
        actual_tick = core.tick

        def slow_tick(*args):
            result = actual_tick(*args)
            core.scan = type(core.scan)(**{**vars(core.scan), 'received': time.monotonic() - 1})
            return result
        with patch.object(core, 'tick', side_effect=slow_tick):
            node.enhanced.tick()
        self.assertEqual(core.previous.v, 0)
        self.assertEqual(core.input_fault, 'scan_timeout_during_planning')

    def test_real_bridge_timers_handshake_dry_run_and_command_expiry(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ws/src/esp_bridge/test'))
        from test_serial_safety import SerialPort
        for dry in (True, False):
            with patch('esp_bridge.cmd_vel_to_serial.serial.Serial', SerialPort):
                node = CmdVelToSerial(
                    parameter_overrides=[
                        Parameter(
                            'dry_run', value=dry), Parameter(
                            'cmd_vel_topic', value='/test_serial_cmd')])
            self.nodes.append(node)
            self.executor.add_node(node)
            self.assertEqual(node.ser.written, ['STOP\n'])
            node.ser.incoming = b'ACK STOP\n'
            self.spin(.1)
            pub = self.probe.create_publisher(Twist, '/test_serial_cmd', 10)
            msg = Twist()
            msg.linear.x = .05
            msg.angular.z = .25
            self.spin(.4, lambda: pub.publish(msg))
            expected = 'CMD v 0.000 w 0.000\n' if dry else 'CMD v 0.050 w 0.250\n'
            self.assertIn(expected, node.ser.written)
            self.spin(.4)
            self.assertEqual(node.ser.written[-1], 'CMD v 0.000 w 0.000\n')
            node.ser.ack_on_stop = True
            node.close()
            self.assertEqual(node.ser.written[-1], 'STOP\n')
            self.executor.remove_node(node)
            self.nodes.remove(node)
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
