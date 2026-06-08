#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String


class SafetyMuxNode(Node):
    def __init__(self):
        super().__init__('safety_mux_node')

        self.declare_parameter('camera_timeout_sec', 0.5)
        self.declare_parameter('vfh_timeout_sec', 0.5)
        self.declare_parameter('prefer_camera_when_target_seen', True)

        self.camera_timeout_sec = float(self.get_parameter('camera_timeout_sec').value)
        self.vfh_timeout_sec = float(self.get_parameter('vfh_timeout_sec').value)
        self.prefer_camera = bool(self.get_parameter('prefer_camera_when_target_seen').value)

        self.camera_cmd = Twist()
        self.vfh_cmd = Twist()

        self.camera_seen = False
        self.critical_stop = False
        self.speed_limit = 1.0

        self.last_camera_time = self.get_clock().now()
        self.last_vfh_time = self.get_clock().now()
        self.last_seen_time = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel_camera', self.on_camera_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_vfh', self.on_vfh_cmd, 10)
        self.create_subscription(Bool, '/vision/target_seen', self.on_target_seen, 10)
        self.create_subscription(Bool, '/fusion/critical_stop', self.on_critical_stop, 10)
        self.create_subscription(Float32, '/fusion/speed_limit', self.on_speed_limit, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(String, '/mux/debug', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Safety mux node started. Publishing final /cmd_vel')

    def on_camera_cmd(self, msg):
        self.camera_cmd = msg
        self.last_camera_time = self.get_clock().now()

    def on_vfh_cmd(self, msg):
        self.vfh_cmd = msg
        self.last_vfh_time = self.get_clock().now()

    def on_target_seen(self, msg):
        self.camera_seen = msg.data
        if msg.data:
            self.last_seen_time = self.get_clock().now()

    def on_critical_stop(self, msg):
        self.critical_stop = msg.data

    def on_speed_limit(self, msg):
        self.speed_limit = max(0.0, min(1.0, float(msg.data)))

    def stop_cmd(self):
        return Twist()

    def clamp_speed(self, cmd):
        out = Twist()

        max_v = abs(cmd.linear.x) * self.speed_limit

        if cmd.linear.x > 0.0:
            out.linear.x = min(cmd.linear.x, max_v)
        elif cmd.linear.x < 0.0:
            out.linear.x = max(cmd.linear.x, -max_v)
        else:
            out.linear.x = 0.0

        out.angular.z = cmd.angular.z
        return out

    def control_loop(self):
        now = self.get_clock().now()

        camera_age = (now - self.last_camera_time).nanoseconds / 1e9
        vfh_age = (now - self.last_vfh_time).nanoseconds / 1e9
        seen_age = (now - self.last_seen_time).nanoseconds / 1e9

        final_cmd = Twist()
        mode = 'stop'

        if self.critical_stop:
            final_cmd = self.stop_cmd()
            mode = 'critical_stop'

        elif (
            self.prefer_camera
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
            f"mode:{mode}|critical_stop:{int(self.critical_stop)}|"
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
