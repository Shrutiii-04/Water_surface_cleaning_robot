#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FakeLiDAR(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz

        self.get_logger().info("Fake LiDAR node started")

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"

        # LiDAR settings
        scan.angle_min = -math.pi / 2      # -90 degrees
        scan.angle_max = math.pi / 2       # +90 degrees
        scan.angle_increment = math.pi / 180  # 1 degree resolution
        scan.range_min = 0.0
        scan.range_max = 5.0

        # Create fake ranges (all clear: 5m)
        num_samples = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [5.0] * num_samples

        # Simulate a front obstacle at 1 meter
        front_index = num_samples // 2
        scan.ranges[front_index] = 1.0

        self.publisher_.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLiDAR()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
