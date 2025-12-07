import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ObstacleMarker(Node):
    def __init__(self):
        super().__init__('obstacle_marker')
        self.pub = self.create_publisher(Marker, '/obstacles', 10)
        self.timer = self.create_timer(1.0, self.publish_obstacle)

    def publish_obstacle(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.2
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.5
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        self.pub.publish(marker)
        self.get_logger().info("Obstacle published to RViz")

def main():
    rclpy.init()
    node = ObstacleMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
