import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.base_frame = self.declare_parameter('base_frame', 'boat_1/hull_1').value
        self.odom_frame = self.declare_parameter('odom_frame', 'world').value
        
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.sub = self.create_subscription(
            Odometry, 
            '/boat_1/odom', 
            self.cb, 
            qos
        )
        
        self.get_logger().info(f'PoseToTF: {self.odom_frame} -> {self.base_frame} (from /boat_1/odom)')

    def cb(self, msg: Odometry):
        t = TransformStamped()
        
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
