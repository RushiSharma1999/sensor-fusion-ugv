#!/usr/bin/env python3
import math
import time
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # Params
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('rx_hz', 50.0)
        self.declare_parameter('debug_serial', False)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('tof_front_topic', '/tof/front/summary')
        self.declare_parameter('ultra_raw_topic', '/esp/ultrasonic_raw')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.rx_hz = float(self.get_parameter('rx_hz').value)
        self.debug_serial = bool(self.get_parameter('debug_serial').value)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        tof_front_topic = self.get_parameter('tof_front_topic').value
        ultra_raw_topic = self.get_parameter('ultra_raw_topic').value

        self.ser = serial.Serial(port, baud, timeout=0.0)
        self.get_logger().info(f"Serial open: {port} @ {baud}")

        # cmd_vel in
        self.last_twist = Twist()
        self.last_rx_time = time.time()
        self.sub = self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)

        # odom out + TF
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # sensor outputs
        self.tof_front_pub = self.create_publisher(String, tof_front_topic, 10)
        self.ultra_raw_pub = self.create_publisher(String, ultra_raw_topic, 10)

        # Serial RX line buffer
        self._rx_buf = ""

        # Timers
        self.cmd_timer = self.create_timer(1.0 / self.rate_hz, self.on_cmd_timer)
        self.rx_timer = self.create_timer(1.0 / self.rx_hz, self.on_rx_timer)

        self.get_logger().info(
            f"Bridge running: {cmd_vel_topic} -> serial, serial -> {odom_topic}, "
            f"{tof_front_topic}, {ultra_raw_topic}"
        )

    def on_cmd_vel(self, msg: Twist):
        self.last_twist = msg
        self.last_rx_time = time.time()

    def on_cmd_timer(self):
        if time.time() - self.last_rx_time > 0.25:
            v = 0.0
            w = 0.0
        else:
            v = float(self.last_twist.linear.x)
            w = float(self.last_twist.angular.z)

        line = f"CMD v {v:.3f} w {w:.3f}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def on_rx_timer(self):
        try:
            data = self.ser.read(4096)
            if not data:
                return
            self._rx_buf += data.decode('ascii', errors='ignore')
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
            return

        while '\n' in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split('\n', 1)
            line = line.strip()
            if not line:
                continue

            if self.debug_serial:
                self.get_logger().info(f"SERIAL: {line}")

            if line.startswith("ODOM"):
                self.handle_odom_line(line)
            elif line.startswith("TOF|"):
                self.handle_tof_line(line)
            elif line.startswith("ULTRA|"):
                self.handle_ultra_line(line)
            # ignore everything else

    def handle_odom_line(self, line: str):
        try:
            parts = line.split()

            def get_val(label: str) -> float:
                i = parts.index(label)
                return float(parts[i + 1])

            x = get_val("x")
            y = get_val("y")
            th = get_val("th")
            v = get_val("v")
            w = get_val("w")
        except Exception:
            self.get_logger().warn(f"Bad ODOM line: {line}")
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(th)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def handle_tof_line(self, line: str):
        try:
            parts = line.split('|')
            if parts[0] != "TOF":
                self.get_logger().warn(f"Bad TOF prefix: {line}")
                return

            fields = {}
            for part in parts[1:]:
                if ':' not in part:
                    continue
                key, value = part.split(':', 1)
                fields[key] = value

            sensor_id = fields.get("ID", "")
            if sensor_id != "FRONT":
                return

            timestamp_ms = int(fields.get("T", "0"))
            valid = int(fields.get("VALID", "0"))
            center = float(fields.get("CENTER", "0.0"))
            near_min = float(fields.get("NEAR_MIN", "0.0"))
            near_avg = float(fields.get("NEAR_AVG", "0.0"))
            far_avg = float(fields.get("FAR_AVG", "0.0"))
        except Exception:
            self.get_logger().warn(f"Bad TOF line: {line}")
            return

        msg = String()
        msg.data = (
            f"ID:{sensor_id}|T:{timestamp_ms}|VALID:{valid}|"
            f"CENTER:{center:.1f}|NEAR_MIN:{near_min:.1f}|"
            f"NEAR_AVG:{near_avg:.1f}|FAR_AVG:{far_avg:.1f}"
        )
        self.tof_front_pub.publish(msg)

    def handle_ultra_line(self, line: str):
        """
        Expected:
        ULTRA|T:12346|FRONT:22.6|LEFT:41.8|RIGHT:39.5|BACK:55.2|VALID_FRONT:1|VALID_LEFT:1|VALID_RIGHT:1|VALID_BACK:1
        """
        try:
            parts = line.split('|')
            if parts[0] != "ULTRA":
                self.get_logger().warn(f"Bad ULTRA prefix: {line}")
                return

            fields = {}
            for part in parts[1:]:
                if ':' not in part:
                    continue
                key, value = part.split(':', 1)
                fields[key] = value

            timestamp_ms = int(fields.get("T", "0"))
            front = float(fields.get("FRONT", "400.0"))
            left = float(fields.get("LEFT", "400.0"))
            right = float(fields.get("RIGHT", "400.0"))
            back = float(fields.get("BACK", "400.0"))

            valid_front = int(fields.get("VALID_FRONT", "0"))
            valid_left = int(fields.get("VALID_LEFT", "0"))
            valid_right = int(fields.get("VALID_RIGHT", "0"))
            valid_back = int(fields.get("VALID_BACK", "0"))
        except Exception:
            self.get_logger().warn(f"Bad ULTRA line: {line}")
            return

        msg = String()
        msg.data = (
            f"T:{timestamp_ms}|FRONT:{front:.1f}|LEFT:{left:.1f}|RIGHT:{right:.1f}|BACK:{back:.1f}|"
            f"VALID_FRONT:{valid_front}|VALID_LEFT:{valid_left}|VALID_RIGHT:{valid_right}|VALID_BACK:{valid_back}"
        )
        self.ultra_raw_pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
