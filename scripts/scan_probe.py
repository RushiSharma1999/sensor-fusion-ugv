#!/usr/bin/env python3
"""Read-only /scan diagnostics, once a second. Ctrl+C to exit."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self):
        super().__init__('ugv_readonly_scan_probe')
        self.previous = None
        self.count = 0
        self.no_front_frames = 0
        self.last_has_front = None
        self.last_log = 0.0
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.check)

    def on_scan(self, msg):
        self.previous = time.monotonic()
        self.count += 1
        valid = lambda r: math.isfinite(r) and r > 0 and msg.range_min <= r <= msg.range_max
        front = [r for i, r in enumerate(msg.ranges)
                 if abs(msg.angle_min + i * msg.angle_increment) <= math.radians(15)]
        finite = [r for r in front if valid(r)]
        has_front = bool(finite)
        if not has_front:
            self.no_front_frames += 1
        transition = self.last_has_front is not None and has_front != self.last_has_front
        self.last_has_front = has_front
        if self.previous - self.last_log < 1 and not transition:
            return
        self.last_log = self.previous
        nearest = min(finite) if finite else float('nan')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        self.get_logger().info(
            f'frame={msg.header.frame_id!r} stamp_age={age:.3f}s count={self.count} '
            f'bins={len(msg.ranges)} finite_total={sum(valid(r) for r in msg.ranges)} '
            f'front_finite={len(finite)}/{len(front)} front_min={nearest:.3f}m '
            f'no_front_frames={self.no_front_frames}/{self.count} '
            f'front_inf={sum(math.isinf(r) for r in front)} front_nan={sum(math.isnan(r) for r in front)}')

    def check(self):
        if self.previous is None or time.monotonic() - self.previous > 1.0:
            self.get_logger().warn('No /scan received in the last second')


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
