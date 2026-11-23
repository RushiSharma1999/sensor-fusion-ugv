"""Exercise the installed ROS launch with synthetic cloud and ESP32 serial data."""

import math
import os
import pty
import re
import signal
import struct
import subprocess
import tempfile
import time
import unittest
from contextlib import contextmanager
from unittest.mock import patch

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField
    from std_msgs.msg import Bool, Float32, Float32MultiArray
except ImportError:
    rclpy = None


@unittest.skipUnless(rclpy, 'requires a sourced ROS 2 workspace')
class FusedStackTest(unittest.TestCase):
    @contextmanager
    def stack(self, motion_enabled, *, use_tof=False):
        with tempfile.TemporaryFile() as log:
            master, slave = pty.openpty()
            os.set_blocking(master, False)
            process = subprocess.Popen(
                [
                    'ros2', 'launch', 'fusion_pkg', 'fused_stack.launch.py',
                    'start_driver:=false', f'serial_port:={os.ttyname(slave)}',
                    f'motion_enabled:={str(motion_enabled).lower()}',
                    f'use_tof:={str(use_tof).lower()}',
                ],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            self.master = master
            self.serial_output = b''
            self.process = process
            try:
                yield
            except AssertionError as error:
                log.seek(0)
                self.fail(f'{error}\nROS launch output:\n{log.read().decode(errors="replace")[-7000:]}')
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                os.close(slave)
                os.close(master)

    def setUp(self):
        isolated_domain = patch.dict(os.environ, {
            'ROS_LOCALHOST_ONLY': '1',
            'ROS_DOMAIN_ID': str(100 + os.getpid() % 70),
        })
        isolated_domain.start()
        self.addCleanup(isolated_domain.stop)
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('fused_stack_test')
        self.addCleanup(self.node.destroy_node)
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/unilidar/cloud', 10)
        self.latest = {}
        for msg_type, topic, qos in (
            (LaserScan, '/scan', qos_profile_sensor_data),
            (Twist, '/cmd_vel', 10),
            (Twist, '/cmd_vel_vfh', 10),
            (Bool, '/fusion/critical_stop', 10),
            (Float32, '/fusion/speed_limit', 10),
            (Float32MultiArray, '/fusion/sector_penalties', 10),
        ):
            self.node.create_subscription(
                msg_type, topic, lambda msg, key=topic: self.latest.update({key: msg}), qos)
        self.counter = 0

    def cloud(self, radius):
        cloud = PointCloud2()
        cloud.header.frame_id = 'unilidar_lidar'
        cloud.header.stamp = self.node.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = 360
        cloud.fields = [
            PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(('x', 'y', 'z'))
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(
            struct.pack('<fff', radius * math.cos(a), radius * math.sin(a), 0.0)
            for a in (math.radians(i) for i in range(cloud.width))
        )
        self.cloud_pub.publish(cloud)

    def tick(self, *, radius=3.0, sonar=(65, 65, 65), tof=False):
        self.assertIsNone(self.process.poll(), 'ROS launch exited')
        if radius is not None:
            self.cloud(radius)
        self.counter += 1
        data = ''
        if sonar is not None:
            for name, distance in zip(('LEFT', 'FRONT', 'RIGHT'), sonar):
                distance_field = 400 if distance is None else distance
                data += (f'US|ID:{name}|T:{self.counter}|VALID:{int(distance is not None)}'
                         f'|CM:{distance_field}\n')
        if tof:
            data += (f'TOF|ID:FRONT|T:{self.counter}|VALID:1|CENTER:40|NEAR_MIN:20'
                     '|NEAR_AVG:25|FAR_AVG:60\n')
        if data:
            os.write(self.master, data.encode())
        try:
            self.serial_output += os.read(self.master, 4096)
        except BlockingIOError:
            pass
        end = time.monotonic() + 0.06
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for(self, description, condition, **inputs):
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            self.tick(**inputs)
            if condition():
                return
        self.fail(description)

    def command(self, topic='/cmd_vel'):
        msg = self.latest.get(topic)
        return None if msg is None else (msg.linear.x, msg.angular.z)

    def data(self, topic, default=None):
        return getattr(self.latest.get(topic), 'data', default)

    def test_cloud_sonar_fusion_to_serial_and_stops(self):
        with self.stack(motion_enabled=True):
            self.wait_for('clear LiDAR and sonar did not yield forward command',
                          lambda: '/scan' in self.latest and (self.command() or (0.0, 0.0))[0] > 0)
            self.assertEqual(
                [pub.node_name for pub in self.node.get_publishers_info_by_topic('/cmd_vel')],
                ['safety_mux_node'],
            )
            self.assertEqual(self.latest['/scan'].header.frame_id, 'unilidar_lidar')
            self.wait_for('serial bridge did not forward a positive velocity', lambda: re.search(
                rb'CMD v 0\.0[1-9][0-9] w ', self.serial_output))

            self.wait_for(
                'left sonar did not apply a directional penalty and slow the final command',
                lambda: (
                    max(self.data('/fusion/sector_penalties', [])[17:], default=0) > 0
                    and max(self.data('/fusion/sector_penalties', [])[:16], default=0) == 0
                    and 0 < self.command()[0] < 0.05
                ),
                sonar=(22, 65, 65),
            )
            self.serial_output = b''
            self.wait_for(
                'a near front sonar return did not stop both axes and reach serial',
                lambda: (
                    self.data('/fusion/critical_stop')
                    and self.command() == (0.0, 0.0)
                    and b'CMD v 0.000 w 0.000\n' in self.serial_output
                ),
                sonar=(65, 10, 65),
            )
            self.wait_for('command did not recover with clear fresh sonar',
                          lambda: self.command()[0] > 0)
            self.wait_for('missing LiDAR did not stop the final command',
                          lambda: self.command() == (0.0, 0.0), radius=None)
            self.wait_for('command did not recover with a fresh cloud',
                          lambda: self.command()[0] > 0)
            self.wait_for(
                'a near LiDAR return did not stop while sonar stayed clear',
                lambda: self.data('/fusion/critical_stop') is False and self.command() == (0.0, 0.0),
                radius=0.3,
            )
            self.wait_for('command did not recover with a clear cloud',
                          lambda: self.command()[0] > 0)
            self.wait_for(
                'a cloud filtered out by the converter did not stop the final command',
                lambda: self.data('/fusion/critical_stop') is False and self.command() == (0.0, 0.0),
                radius=21.0,
            )
            self.wait_for('command did not recover after the filtered cloud',
                          lambda: self.command()[0] > 0)
            self.wait_for(
                'explicitly invalid sonar did not stop while LiDAR was clear',
                lambda: self.data('/fusion/critical_stop') and self.command() == (0.0, 0.0),
                sonar=(None, None, None),
            )
            self.wait_for('command did not recover with valid sonar',
                          lambda: self.command()[0] > 0)
            self.wait_for(
                'stale sonar did not stop motion while LiDAR was clear',
                lambda: self.data('/fusion/critical_stop') and self.command() == (0.0, 0.0),
                sonar=None,
            )

    def test_stationary_mode_keeps_final_output_zero(self):
        with self.stack(motion_enabled=False):
            self.wait_for('expected a nonzero candidate with the final command disabled', lambda: (
                (self.command('/cmd_vel_vfh') or (0.0, 0.0))[0] > 0
                and self.command() == (0.0, 0.0)
                and b'CMD v 0.000 w 0.000\n' in self.serial_output
            ))
            self.assertNotRegex(self.serial_output, rb'CMD v 0\.0[1-9][0-9]')

    def test_optional_tof_stops_when_its_data_goes_stale(self):
        with self.stack(motion_enabled=True, use_tof=True):
            self.wait_for('all three fresh sensors did not yield a forward command', lambda: (
                (self.command() or (0.0, 0.0))[0] > 0
            ), tof=True)
            self.wait_for('enabled ToF going stale did not stop the final command', lambda: (
                self.data('/fusion/critical_stop') and self.command() == (0.0, 0.0)
            ))


if __name__ == '__main__':
    unittest.main(verbosity=2)
