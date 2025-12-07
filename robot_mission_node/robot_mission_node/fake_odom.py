# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from math import cos, sin

# class FakeOdom(Node):
#     def __init__(self):
#         super().__init__('fake_odom')
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

#         # Initialize robot pose at origin
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0

#         # Velocity commands
#         self.v = 0.0
#         self.w = 0.0

#         # Timer to update odometry at 10 Hz
#         self.timer = self.create_timer(0.1, self.update)

#     def cmd_cb(self, msg):
#         self.v = msg.linear.x
#         self.w = msg.angular.z

#     def update(self):
#         dt = 0.1
#         # Simple differential drive kinematics
#         self.x += self.v * cos(self.yaw) * dt
#         self.y += self.v * sin(self.yaw) * dt
#         self.yaw += self.w * dt

#         # Publish odometry
#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation.z = sin(self.yaw / 2.0)
#         odom.pose.pose.orientation.w = cos(self.yaw / 2.0)

#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FakeOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        self.br = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        self.timer = self.create_timer(0.05, self.update)

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        dt = 0.05

        self.x += self.v * cos(self.yaw) * dt
        self.y += self.v * sin(self.yaw) * dt
        self.yaw += self.w * dt
        

        qz = sin(self.yaw / 2)
        qw = cos(self.yaw / 2)

        # ---- TF ----
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

        # ---- ODOM ----
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.pub.publish(odom)


def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
