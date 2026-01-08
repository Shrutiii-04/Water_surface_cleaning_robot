#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Publish velocity commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, '/cleaning_coverage_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.path = []
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.index = 0
        self.goal_tolerance = 0.25
        self.timer = self.create_timer(0.05, self.control)

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.index = 0
        self.get_logger().info(f"Received path with {len(self.path)} points")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # Convert quaternion to yaw (2D motion)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

    def control(self):
        if len(self.path) == 0 or self.index >= len(self.path):
            self.cmd_pub.publish(Twist())            # publish zero to stop the robot at end
            return

        tx, ty = self.path[self.index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - self.yaw)

        cmd = Twist()
        if dist > self.goal_tolerance:
            cmd.linear.x = min(0.6, 0.6 * (dist / 2.0))
            cmd.angular.z = max(-1.5, min(1.5, 2.5 * yaw_err))
        else:
            self.index += 1

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
