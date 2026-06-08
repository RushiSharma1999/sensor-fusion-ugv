#!/usr/bin/env python3
import serial
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Int32MultiArray


class VisionUartReceiver(Node):
    def __init__(self):
        super().__init__('vision_uart_receiver')

        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.2)
        self.declare_parameter('target_timeout_sec', 1.5)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)

        self.direction_pub = self.create_publisher(String, '/vision/direction', 10)
        self.stop_pub = self.create_publisher(Bool, '/vision/stop_active', 10)
        self.target_seen_pub = self.create_publisher(Bool, '/vision/target_seen', 10)
        self.angle_pub = self.create_publisher(Float32, '/vision/target_angle', 10)
        self.conf_pub = self.create_publisher(Float32, '/vision/target_confidence', 10)
        self.box_pub = self.create_publisher(Int32MultiArray, '/vision/target_box', 10)
        self.debug_pub = self.create_publisher(String, '/vision/debug', 10)

        self.last_target_time = time.time()
        self.target_seen = False

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )

        self.get_logger().info(
            f'Vision UART receiver ready on {self.port} @ {self.baudrate}'
        )

        # Slower timer, matching your original working style
        self.timer = self.create_timer(0.02, self.read_serial)
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)

    def publish_debug(self, text):
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)

    def publish_seen(self, seen):
        msg = Bool()
        msg.data = bool(seen)
        self.target_seen_pub.publish(msg)

    def handle_direction(self, msg):
        msg = msg.strip()

        if len(msg) != 2:
            return

        horizontal = msg[0]
        stop_char = msg[1]

        direction = String()
        stop = Bool()

        if horizontal == 'R':
            direction.data = 'right'
        elif horizontal == 'L':
            direction.data = 'left'
        elif horizontal == 'C':
            direction.data = 'center'
        else:
            direction.data = 'unknown'

        stop.data = stop_char == 'S'

        self.direction_pub.publish(direction)
        self.stop_pub.publish(stop)

        self.publish_debug(
            f"DIR|raw:{msg}|direction:{direction.data}|stop:{int(stop.data)}"
        )

    def parse_data_line(self, line):
        parts = line.split(',')

        if len(parts) != 7:
            self.publish_debug(f"BAD_DET|field_count:{len(parts)}|raw:{line}")
            return

        try:
            class_id = int(float(parts[0]))
            x_min = int(float(parts[1]))
            y_min = int(float(parts[2]))
            x_max = int(float(parts[3]))
            y_max = int(float(parts[4]))
            conf = float(parts[5])
            angle = float(parts[6])

            # Jetson sends 87 instead of 0.87
            if conf > 1.0:
                conf = conf / 100.0

        except Exception as e:
            self.publish_debug(f"BAD_DET|parse_error:{e}|raw:{line}")
            return

        self.last_target_time = time.time()
        self.target_seen = True
        self.publish_seen(True)

        angle_msg = Float32()
        angle_msg.data = angle
        self.angle_pub.publish(angle_msg)

        conf_msg = Float32()
        conf_msg.data = conf
        self.conf_pub.publish(conf_msg)

        box_msg = Int32MultiArray()
        box_msg.data = [class_id, x_min, y_min, x_max, y_max]
        self.box_pub.publish(box_msg)

        self.publish_debug(
            f"DET|class:{class_id}|box:{x_min},{y_min},{x_max},{y_max}|"
            f"conf:{conf:.2f}|angle:{angle:.2f}"
        )

    def check_timeout(self):
        if not self.target_seen:
            return

        elapsed = time.time() - self.last_target_time

        if elapsed > self.target_timeout_sec:
            self.target_seen = False
            self.publish_seen(False)
            self.publish_debug(f"TARGET_LOST|elapsed:{elapsed:.2f}")

    def read_serial(self):
        try:
            msg = self.ser.readline().decode("ascii", errors="ignore").strip()

            if not msg:
                return

            # Same logic as your original working parser
            if len(msg) == 2 and "," not in msg:
                self.handle_direction(msg)

            elif "," in msg:
                self.parse_data_line(msg)

            elif msg in ("NO_TARGET", "NONE"):
                self.target_seen = False
                self.publish_seen(False)
                self.publish_debug(f"NO_TARGET|raw:{msg}")

            else:
                self.publish_debug(f"UNKNOWN|raw:{msg}")

        except Exception as e:
            self.get_logger().warn(f"UART error: {e}")

    def destroy_node(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionUartReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
