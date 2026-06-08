import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import csv
import time

class LidarLogger(Node):
    def __init__(self):
        super().__init__('lidar_logger')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            qos_profile_sensor_data
        )

        filename = f'lidar_data_{int(time.time())}.csv'
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow(['time', 'angle', 'distance'])

        self.get_logger().info(f"Logging to {filename}")

    def callback(self, msg):
        timestamp = time.time()

        for i, dist in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -0.087 < angle < 0.087:
                self.writer.writerow([timestamp, angle, dist])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
