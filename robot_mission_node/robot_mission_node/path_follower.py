# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import Twist, PoseStamped
# from sensor_msgs.msg import LaserScan
# import numpy as np
# import math

# class PathFollower(Node):
#     def __init__(self):
#         super().__init__('path_follower')

#         # Subscribers
#         self.path_sub = self.create_subscription(Path, '/cleaning_coverage_path', self.path_callback, 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

#         # Publisher
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Robot state
#         self.path = []
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         self.robot_yaw = 0.0
#         self.scan_data = []

#         # Control parameters
#         self.lookahead_distance = 0.5  # meters
#         self.k_linear = 0.5
#         self.k_angular = 1.0
#         self.obstacle_distance_threshold = 0.7  # meters

#         # Timer
#         self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

#         self.get_logger().info("PathFollower node started ")

#     # ---------------- Subscribers ----------------
#     def path_callback(self, msg):
#         self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
#         self.get_logger().info(f'Path received with {len(self.path)} points')

#     def odom_callback(self, msg):
#         self.robot_x = msg.pose.pose.position.x
#         self.robot_y = msg.pose.pose.position.y
#         # yaw from quaternion
#         q = msg.pose.pose.orientation
#         self.robot_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

#     def scan_callback(self, msg):
#         self.scan_data = msg.ranges

#     # ---------------- Control Loop ----------------
#     def control_loop(self):
#         if not self.path:
#             return

#         lookahead_point = self.get_lookahead_point()
#         dx = lookahead_point[0] - self.robot_x
#         dy = lookahead_point[1] - self.robot_y
#         path_angle = math.atan2(dy, dx)
#         angle_error = self.normalize_angle(path_angle - self.robot_yaw)

#         linear_vel = self.k_linear
#         angular_vel = self.k_angular * angle_error

#         # Simple obstacle avoidance
#         if self.scan_data and min(self.scan_data) < self.obstacle_distance_threshold:
#             linear_vel = 0.0
#             angular_vel = 0.5
#             self.get_logger().warn(f"Obstacle detected! Turning on spot.")

#         # Publish Twist
#         cmd = Twist()
#         cmd.linear.x = linear_vel
#         cmd.angular.z = angular_vel
#         self.cmd_pub.publish(cmd)

#         # Log control command
#         self.get_logger().info(f'Controlling robot: linear={linear_vel:.2f}, angular={angular_vel:.2f}')

#     # ---------------- Helpers ----------------
#     def get_lookahead_point(self):
#         # Return the first path point farther than lookahead_distance
#         for x, y in self.path:
#             dist = math.hypot(x - self.robot_x, y - self.robot_y)
#             if dist >= self.lookahead_distance:
#                 return (x, y)
#         return self.path[-1]  # last point if none farther

#     def normalize_angle(self, angle):
#         while angle > math.pi:
#             angle -= 2*math.pi
#         while angle < -math.pi:
#             angle += 2*math.pi
#         return angle

# def main(args=None):
#     rclpy.init(args=args)
#     node = PathFollower()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # subscribe to the planner's topic
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
        # Convert quaternion -> yaw (robust, no external dependency)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

    def control(self):
        if len(self.path) == 0 or self.index >= len(self.path):
            # publish zero to stop the robot at end
            self.cmd_pub.publish(Twist())
            return

        tx, ty = self.path[self.index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - self.yaw)

        cmd = Twist()
        # simple proportional controller
        if dist > self.goal_tolerance:
            # linearly scale forward speed by distance (clamped)
            cmd.linear.x = min(0.6, 0.6 * (dist / 2.0))
            # angular velocity proportional to heading error
            cmd.angular.z = max(-1.5, min(1.5, 2.5 * yaw_err))
        else:
            # reached this waypoint
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
