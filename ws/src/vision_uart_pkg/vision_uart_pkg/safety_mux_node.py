#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String


def inspect_scan(msg, now_sec, max_age, half_fov):
    """Finite returns are evidence; no-return bins are unknown, never clear.

    This checks message structure/freshness, not physical frame alignment or
    whether a projected scan represents the full robot footprint.
    """
    if not msg.ranges:
        return False, 'empty_scan'
    values = (msg.angle_min, msg.angle_max, msg.angle_increment,
              msg.range_min, msg.range_max)
    if not all(math.isfinite(x) for x in values):
        return False, 'nonfinite_metadata'
    if msg.angle_increment <= 0 or msg.range_min < 0 or msg.range_max <= msg.range_min:
        return False, 'bad_metadata'
    end = msg.angle_min + (len(msg.ranges) - 1) * msg.angle_increment
    if abs(end - msg.angle_max) > max(1e-5, 1.5 * msg.angle_increment):
        return False, 'range_count_mismatch'
    if msg.angle_min > -half_fov + msg.angle_increment or end < half_fov - msg.angle_increment:
        return False, 'insufficient_fov'
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if stamp <= 0 or not -0.1 <= now_sec - stamp <= max_age:
        return False, 'stale_or_bad_stamp'
    front_count = 0
    for i, value in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        if (abs(angle) <= math.radians(15.0) and math.isfinite(value)
                and value > 0 and msg.range_min <= value <= msg.range_max):
            front_count += 1
    if front_count == 0:
        return False, 'no_finite_front_returns'
    return True, 'ok'


class SafetyMuxNode(Node):
    def __init__(self):
        super().__init__('safety_mux_node')

        self.declare_parameter('camera_timeout_sec', 0.5)
        self.declare_parameter('vfh_timeout_sec', 0.5)
        self.declare_parameter('prefer_camera_when_target_seen', False)
        self.declare_parameter('lidar_only_mode', True)
        self.declare_parameter('motion_enabled', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout_sec', 0.25)
        self.declare_parameter('max_scan_age_sec', 0.5)
        self.declare_parameter('front_fov_deg', 160.0)
        self.declare_parameter('front_stop_dist', 0.34)
        self.declare_parameter('max_linear_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('fusion_timeout_sec', 0.5)
        self.lidar_only = bool(self.get_parameter('lidar_only_mode').value)
        self.motion_enabled = bool(self.get_parameter('motion_enabled').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self.max_scan_age = float(self.get_parameter('max_scan_age_sec').value)
        self.half_fov = math.radians(float(self.get_parameter('front_fov_deg').value)) / 2
        self.front_stop_dist = float(self.get_parameter('front_stop_dist').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.fusion_timeout = float(self.get_parameter('fusion_timeout_sec').value)
        self.last_scan_time = None
        self.last_scan_stamp = None
        self.scan_valid = False
        self.scan_reason = 'waiting_for_scan'
        self.last_stop_time = None
        self.last_speed_time = None

        self.camera_timeout_sec = float(self.get_parameter('camera_timeout_sec').value)
        self.vfh_timeout_sec = float(self.get_parameter('vfh_timeout_sec').value)
        self.prefer_camera = bool(self.get_parameter('prefer_camera_when_target_seen').value)

        self.camera_cmd = Twist()
        self.vfh_cmd = Twist()

        self.camera_seen = False
        self.critical_stop = False
        self.speed_limit = 1.0

        for value in (self.scan_timeout, self.max_scan_age, self.max_linear,
                      self.max_angular, self.fusion_timeout, self.camera_timeout_sec,
                      self.vfh_timeout_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Timeouts and command limits must be finite and positive')
        if not math.isfinite(self.half_fov) or not math.radians(15) <= self.half_fov <= math.pi:
            raise ValueError('Invalid scan field of view')
        if not math.isfinite(self.front_stop_dist) or self.front_stop_dist < 0:
            raise ValueError('Invalid front stop distance')
        self.last_camera_time = None
        self.last_vfh_time = None
        self.last_seen_time = None
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)

        self.create_subscription(Twist, '/cmd_vel_camera', self.on_camera_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_vfh', self.on_vfh_cmd, 10)
        self.create_subscription(Bool, '/vision/target_seen', self.on_target_seen, 10)
        self.create_subscription(Bool, '/fusion/critical_stop', self.on_critical_stop, 10)
        self.create_subscription(Float32, '/fusion/speed_limit', self.on_speed_limit, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(String, '/mux/debug', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Safety mux node started. Publishing final /cmd_vel')

    @staticmethod
    def finite_cmd(msg):
        return all(math.isfinite(x) for x in
                   (msg.linear.x, msg.linear.y, msg.linear.z,
                    msg.angular.x, msg.angular.y, msg.angular.z))

    def on_scan(self, msg):
        self.last_scan_time = time.monotonic()
        valid, reason = inspect_scan(msg, self.get_clock().now().nanoseconds * 1e-9,
                                     self.max_scan_age, self.half_fov)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if valid and self.last_scan_stamp is not None and stamp <= self.last_scan_stamp:
            valid, reason = False, 'nonadvancing_scan_stamp'
        if valid:
            self.last_scan_stamp = stamp
            front = [r for i, r in enumerate(msg.ranges)
                     if abs(msg.angle_min + i * msg.angle_increment) <= math.radians(15)
                     and math.isfinite(r) and r > 0 and msg.range_min <= r <= msg.range_max]
            if min(front) <= self.front_stop_dist:
                valid, reason = False, 'front_stop'
        self.scan_valid, self.scan_reason = valid, reason
        if not valid:
            self.cmd_pub.publish(Twist())

    def on_camera_cmd(self, msg):
        self.camera_cmd = msg if self.finite_cmd(msg) else Twist()
        self.last_camera_time = time.monotonic()

    def on_vfh_cmd(self, msg):
        self.vfh_cmd = msg if self.finite_cmd(msg) else Twist()
        self.last_vfh_time = time.monotonic()

    def on_target_seen(self, msg):
        self.camera_seen = msg.data
        if msg.data:
            self.last_seen_time = time.monotonic()

    def on_critical_stop(self, msg):
        self.critical_stop = msg.data
        self.last_stop_time = time.monotonic()
        if self.critical_stop and not self.lidar_only:
            self.last_vfh_time = None
            self.cmd_pub.publish(Twist())

    def on_speed_limit(self, msg):
        value = float(msg.data)
        self.speed_limit = max(0.0, min(1.0, value)) if math.isfinite(value) else 0.0
        self.last_speed_time = time.monotonic()

    def stop_cmd(self):
        return Twist()

    def clamp_speed(self, cmd):
        out = Twist()
        if not self.finite_cmd(cmd):
            return out
        limit = 1.0 if self.lidar_only else self.speed_limit
        if limit <= 0:
            return out
        # This profile permits only forward travel. Scaling both components
        # preserves curvature when enforcing absolute command limits.
        v, w = max(0.0, cmd.linear.x), cmd.angular.z
        factor = min(1.0, self.max_linear / max(abs(v), 1e-12),
                     self.max_angular / max(abs(w), 1e-12))
        out.linear.x = v * factor * limit
        out.angular.z = w * factor * limit
        return out

    @staticmethod
    def age(now, last):
        return float('inf') if last is None else max(0.0, now - last)

    def control_loop(self):
        now = time.monotonic()

        camera_age = self.age(now, self.last_camera_time)
        vfh_age = self.age(now, self.last_vfh_time)
        seen_age = self.age(now, self.last_seen_time)
        scan_age = self.age(now, self.last_scan_time)

        final_cmd = Twist()
        mode = 'stop'

        if not self.scan_valid or scan_age > self.scan_timeout:
            mode = 'scan_guard:' + (self.scan_reason if scan_age <= self.scan_timeout else 'timeout')

        elif not self.lidar_only and (
            self.age(now, self.last_stop_time) > self.fusion_timeout
            or self.age(now, self.last_speed_time) > self.fusion_timeout
        ):
            mode = 'fusion_stale'

        elif not self.lidar_only and (self.critical_stop or self.speed_limit <= 0):
            final_cmd = self.stop_cmd()
            mode = 'critical_stop'

        elif not self.motion_enabled:
            mode = 'motion_disabled'

        elif (
            not self.lidar_only
            and self.prefer_camera
            and self.camera_seen
            and camera_age < self.camera_timeout_sec
            and seen_age < self.camera_timeout_sec
        ):
            final_cmd = self.clamp_speed(self.camera_cmd)
            mode = 'camera_follow'

        elif vfh_age < self.vfh_timeout_sec:
            final_cmd = self.clamp_speed(self.vfh_cmd)
            mode = 'vfh_avoidance'

        else:
            final_cmd = self.stop_cmd()
            mode = 'no_valid_cmd'

        self.cmd_pub.publish(final_cmd)

        dbg = String()
        dbg.data = (
            f"mode:{mode}|lidar_only:{int(self.lidar_only)}|scan_age:{scan_age:.2f}|critical_stop:{int(self.critical_stop)}|"
            f"speed_limit:{self.speed_limit:.2f}|camera_seen:{int(self.camera_seen)}|"
            f"v:{final_cmd.linear.x:.2f}|w:{final_cmd.angular.z:.2f}|"
            f"camera_age:{camera_age:.2f}|vfh_age:{vfh_age:.2f}"
        )
        self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMuxNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
