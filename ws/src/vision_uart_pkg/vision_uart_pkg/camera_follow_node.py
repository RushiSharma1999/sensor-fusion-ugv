#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, String, Int32MultiArray
from geometry_msgs.msg import Twist


class CameraFollowNode(Node):
    def __init__(self):
        super().__init__('camera_follow_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_camera')

        self.declare_parameter('max_linear_speed', 0.40)
        self.declare_parameter('min_linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.32)

        self.declare_parameter('angle_kp', 0.010)
        self.declare_parameter('angle_deadband_deg', 8.0)
        self.declare_parameter('straight_lock_deg', 3.0)
        self.declare_parameter('turn_in_place_angle_deg', 35.0)

        self.declare_parameter('straight_drive_trim_w', 0.0)

        self.declare_parameter('off_center_boost_deg', 12.0)
        self.declare_parameter('off_center_boost_gain', 1.45)

        self.declare_parameter('target_timeout_sec', 2.0)
        self.declare_parameter('lost_grace_sec', 5)
        self.declare_parameter('min_confidence', 0.10)

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('slow_area_ratio', 0.18)
        self.declare_parameter('stop_area_ratio', 0.34)

        self.declare_parameter('cmd_smooth_alpha', 0.12)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.angle_kp = float(self.get_parameter('angle_kp').value)
        self.angle_deadband_deg = float(self.get_parameter('angle_deadband_deg').value)
        self.straight_lock_deg = float(self.get_parameter('straight_lock_deg').value)
        self.turn_in_place_angle_deg = float(self.get_parameter('turn_in_place_angle_deg').value)

        self.straight_drive_trim_w = float(self.get_parameter('straight_drive_trim_w').value)
        self.off_center_boost_deg = float(self.get_parameter('off_center_boost_deg').value)
        self.off_center_boost_gain = float(self.get_parameter('off_center_boost_gain').value)

        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.lost_grace_sec = float(self.get_parameter('lost_grace_sec').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)

        self.slow_area_ratio = float(self.get_parameter('slow_area_ratio').value)
        self.stop_area_ratio = float(self.get_parameter('stop_area_ratio').value)

        self.cmd_smooth_alpha = float(self.get_parameter('cmd_smooth_alpha').value)

        self.target_seen = False
        self.stop_active = False
        self.target_angle = 0.0
        self.target_confidence = 0.0
        self.target_area_ratio = 0.0

        self.prev_v = 0.0
        self.prev_w = 0.0
        self.last_good_angle = 0.0

        self.last_target_time = self.get_clock().now()
        self.lost_started = False
        self.lost_start_time = self.get_clock().now()

        self.create_subscription(Bool, '/vision/target_seen', self.on_target_seen, 10)
        self.create_subscription(Float32, '/vision/target_angle', self.on_target_angle, 10)
        self.create_subscription(Float32, '/vision/target_confidence', self.on_confidence, 10)
        self.create_subscription(Bool, '/vision/stop_active', self.on_stop_active, 10)
        self.create_subscription(Int32MultiArray, '/vision/target_box', self.on_target_box, 10)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(String, '/vision/follow_debug', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Camera follow node publishing to {self.cmd_vel_topic}')

    def on_target_seen(self, msg):
        if msg.data:
            self.target_seen = True
            self.lost_started = False
            self.last_target_time = self.get_clock().now()
        else:
            if self.target_seen and not self.lost_started:
                self.lost_started = True
                self.lost_start_time = self.get_clock().now()
            self.target_seen = False

    def on_target_angle(self, msg):
        self.target_angle = float(msg.data)
        self.last_good_angle = self.target_angle
        self.target_seen = True
        self.lost_started = False
        self.last_target_time = self.get_clock().now()

    def on_confidence(self, msg):
        self.target_confidence = float(msg.data)

    def on_stop_active(self, msg):
        self.stop_active = bool(msg.data)

    def on_target_box(self, msg):
        if len(msg.data) < 5:
            return

        _, x_min, y_min, x_max, y_max = msg.data[:5]

        box_w = max(0, x_max - x_min)
        box_h = max(0, y_max - y_min)
        box_area = box_w * box_h

        image_area = max(1, self.image_width * self.image_height)
        self.target_area_ratio = box_area / image_area

    def clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def smooth(self, old, new):
        a = self.cmd_smooth_alpha
        return (1.0 - a) * old + a * new

    def publish_cmd(self, target_v, target_w, state, elapsed, lost_elapsed=0.0):
        cmd = Twist()

        cmd.linear.x = self.smooth(self.prev_v, target_v)
        cmd.angular.z = self.smooth(self.prev_w, target_w)

        if abs(cmd.linear.x) < 0.008:
            cmd.linear.x = 0.0
        if abs(cmd.angular.z) < 0.008:
            cmd.angular.z = 0.0

        self.prev_v = cmd.linear.x
        self.prev_w = cmd.angular.z

        self.cmd_pub.publish(cmd)

        dbg = String()
        dbg.data = (
            f"{state}|angle:{self.target_angle:.2f}|last_angle:{self.last_good_angle:.2f}|"
            f"conf:{self.target_confidence:.2f}|area:{self.target_area_ratio:.2f}|"
            f"target_seen:{int(self.target_seen)}|lost_elapsed:{lost_elapsed:.2f}|"
            f"target_v:{target_v:.2f}|target_w:{target_w:.2f}|"
            f"v:{cmd.linear.x:.2f}|w:{cmd.angular.z:.2f}|"
            f"trim:{self.straight_drive_trim_w:.3f}|elapsed:{elapsed:.2f}"
        )
        self.debug_pub.publish(dbg)

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_target_time).nanoseconds / 1e9

        if self.stop_active:
            self.publish_cmd(0.0, 0.0, "vision_stop_active", elapsed)
            return

        # Real lost-grace behavior
        if not self.target_seen:
            if not self.lost_started:
                self.lost_started = True
                self.lost_start_time = now

            lost_elapsed = (now - self.lost_start_time).nanoseconds / 1e9

            if lost_elapsed <= self.lost_grace_sec:
                search_w = self.clamp(
                    -0.35 * self.angle_kp * self.last_good_angle,
                    -0.12,
                    0.12
                )

                coast_v = max(0.015, self.prev_v * 0.75)

                self.publish_cmd(
                    coast_v,
                    search_w,
                    "lost_grace_search",
                    elapsed,
                    lost_elapsed
                )
                return

            self.publish_cmd(
                0.0,
                0.0,
                "target_lost_smooth_stop",
                elapsed,
                lost_elapsed
            )
            return

        # If data is stale even though target_seen stayed true, stop smoothly.
        if elapsed > self.target_timeout_sec:
            self.target_seen = False
            self.lost_started = True
            self.lost_start_time = now
            self.publish_cmd(0.0, 0.0, "target_stale_start_lost", elapsed)
            return

        if self.target_confidence < self.min_confidence:
            weak_w = self.clamp(
                -0.25 * self.angle_kp * self.last_good_angle,
                -0.08,
                0.08
            )
            self.publish_cmd(0.015, weak_w, "weak_detection_hold", elapsed)
            return

        angle = self.target_angle
        abs_angle = abs(angle)

        gain = self.angle_kp
        if abs_angle >= self.off_center_boost_deg:
            gain *= self.off_center_boost_gain

        # Negative sign because your robot was turning opposite before.
        target_w = -gain * angle

        if abs_angle <= self.straight_lock_deg:
            target_w = 0.0
            self.prev_w *= 0.25
            state_hint = "straight_lock"

        elif abs_angle <= self.angle_deadband_deg:
            target_w = 0.0
            self.prev_w *= 0.50
            state_hint = "deadband"

        else:
            target_w = self.clamp(
                target_w,
                -self.max_angular_speed,
                self.max_angular_speed
            )
            state_hint = "correcting"

        if self.target_area_ratio >= self.stop_area_ratio:
            target_v = 0.0
            state = "too_close_stop"

        elif abs_angle >= self.turn_in_place_angle_deg:
            target_v = 0.0
            state = "turn_in_place"

        else:
            angle_scale = 1.0 - self.clamp(
                abs_angle / self.turn_in_place_angle_deg,
                0.0,
                1.0
            )

            target_v = self.min_linear_speed + angle_scale * (
                self.max_linear_speed - self.min_linear_speed
            )

            if self.target_area_ratio >= self.slow_area_ratio:
                target_v *= 0.50
                state = f"slow_follow_{state_hint}"
            else:
                state = f"follow_{state_hint}"

        if target_v > 0.02 and abs(target_w) < 0.015:
            target_w += self.straight_drive_trim_w
            state += "_trimmed"

        self.publish_cmd(target_v, target_w, state, elapsed)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0, "shutdown", 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
