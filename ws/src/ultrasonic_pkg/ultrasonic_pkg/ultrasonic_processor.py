#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UltrasonicProcessorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_processor')

        # ---------- Parameters ----------
        self.declare_parameter('ultrasonic_raw_topic', '/esp/ultrasonic_raw')
        self.declare_parameter('ultrasonic_features_topic', '/ultrasonic/features')

        self.declare_parameter('max_range_cm', 50.0)
        self.declare_parameter('min_range_cm', 0.0)
        self.declare_parameter('stale_timeout_sec', 0.5)

        # Optional default bearings for front-left / front-center / front-right
        self.declare_parameter('left_bearing_rad', 0.52)    # about +30 deg
        self.declare_parameter('center_bearing_rad', 0.0)
        self.declare_parameter('right_bearing_rad', -0.52)  # about -30 deg

        self.ultrasonic_raw_topic = self.get_parameter('ultrasonic_raw_topic').value
        self.ultrasonic_features_topic = self.get_parameter('ultrasonic_features_topic').value

        self.max_range_cm = float(self.get_parameter('max_range_cm').value)
        self.min_range_cm = float(self.get_parameter('min_range_cm').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)

        self.left_bearing_rad = float(self.get_parameter('left_bearing_rad').value)
        self.center_bearing_rad = float(self.get_parameter('center_bearing_rad').value)
        self.right_bearing_rad = float(self.get_parameter('right_bearing_rad').value)

        # ---------- State ----------
        self.latest_raw = {}
        self.last_update_time = None

        # ---------- ROS ----------
        self.sub_raw = self.create_subscription(
            String,
            self.ultrasonic_raw_topic,
            self.on_raw_ultrasonic,
            10
        )

        self.features_pub = self.create_publisher(
            String,
            self.ultrasonic_features_topic,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_features)  # 10 Hz

        self.get_logger().info(
            f'Ultrasonic processor started. raw_topic={self.ultrasonic_raw_topic}, '
            f'features_topic={self.ultrasonic_features_topic}'
        )

    # ---------- Parsing ----------
    def parse_fields(self, text: str):
        fields = {}
        for part in text.split('|'):
            if ':' not in part:
                continue
            key, value = part.split(':', 1)
            fields[key.strip().upper()] = value.strip()
        return fields

    def on_raw_ultrasonic(self, msg: String):
        self.latest_raw = self.parse_fields(msg.data)
        self.last_update_time = self.get_clock().now()

    # ---------- Helpers ----------
    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def get_distance_cm(self, data: dict, aliases, default_value=999.0):
        for key in aliases:
            if key in data:
                try:
                    return float(data[key])
                except ValueError:
                    return default_value
        return default_value

    def normalize_distance(self, d_cm: float) -> float:
        """
        Convert distance into hazard in [0, 1].
        0 cm => 1.0 hazard
        >= max_range_cm => 0.0 hazard
        """
        d = self.clamp(d_cm, self.min_range_cm, self.max_range_cm)
        return 1.0 - (d - self.min_range_cm) / max(1e-6, (self.max_range_cm - self.min_range_cm))

    def is_stale(self) -> bool:
        if self.last_update_time is None:
            return True
        age = (self.get_clock().now() - self.last_update_time).nanoseconds * 1e-9
        return age > self.stale_timeout_sec

    # ---------- Core ----------
    def publish_features(self):
        if self.is_stale():
            msg = String()
            msg.data = 'VALID:0|STATE:stale'
            self.features_pub.publish(msg)
            return

        left_cm = self.get_distance_cm(self.latest_raw, ['LEFT', 'L', 'UL_LEFT', 'ULTRA_LEFT', 'FL'])
        center_cm = self.get_distance_cm(self.latest_raw, ['CENTER', 'C', 'UL_CENTER', 'ULTRA_CENTER', 'FC', 'FRONT'])
        right_cm = self.get_distance_cm(self.latest_raw, ['RIGHT', 'R', 'UL_RIGHT', 'ULTRA_RIGHT', 'FR'])

        left_valid = left_cm <= self.max_range_cm
        center_valid = center_cm <= self.max_range_cm
        right_valid = right_cm <= self.max_range_cm

        valid_count = sum([left_valid, center_valid, right_valid])

        if valid_count == 0:
            msg = String()
            msg.data = (
                'VALID:0|STATE:no_echo|'
                f'LEFT_CM:{left_cm:.1f}|CENTER_CM:{center_cm:.1f}|RIGHT_CM:{right_cm:.1f}'
            )
            self.features_pub.publish(msg)
            return

        left_hazard = self.normalize_distance(left_cm) if left_valid else 0.0
        center_hazard = self.normalize_distance(center_cm) if center_valid else 0.0
        right_hazard = self.normalize_distance(right_cm) if right_valid else 0.0

        distances = []
        if left_valid:
            distances.append(('left', left_cm, self.left_bearing_rad, left_hazard))
        if center_valid:
            distances.append(('center', center_cm, self.center_bearing_rad, center_hazard))
        if right_valid:
            distances.append(('right', right_cm, self.right_bearing_rad, right_hazard))

        closest_name, min_cm, strongest_bearing, strongest_hazard = min(distances, key=lambda x: x[1])
        avg_cm = sum(item[1] for item in distances) / len(distances)

        msg = String()
        msg.data = (
            'VALID:1|STATE:ok|'
            f'LEFT_CM:{left_cm:.1f}|CENTER_CM:{center_cm:.1f}|RIGHT_CM:{right_cm:.1f}|'
            f'LEFT_HAZARD:{left_hazard:.2f}|CENTER_HAZARD:{center_hazard:.2f}|RIGHT_HAZARD:{right_hazard:.2f}|'
            f'MIN_CM:{min_cm:.1f}|AVG_CM:{avg_cm:.1f}|'
            f'STRONGEST:{closest_name}|BEARING:{strongest_bearing:.3f}|HAZARD:{strongest_hazard:.2f}'
        )
        self.features_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
