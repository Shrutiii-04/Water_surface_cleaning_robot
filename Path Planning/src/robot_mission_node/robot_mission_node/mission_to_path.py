import requests
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

API_ENDPOINT = "http://127.0.0.1:5000/get_mission"

class MissionToPath(Node):
    def __init__(self):
        super().__init__('mission_to_path')
        self.publisher_ = self.create_publisher(Path, '/cleaning_coverage_path', 10)
        self.timer = self.create_timer(2.0, self.fetch_and_publish_path)

    def fetch_and_publish_path(self):
        try:
            response = requests.get(API_ENDPOINT, timeout=2)
            mission = response.json().get("data", {})

            if not mission or "polygon" not in mission:
                self.get_logger().warn("No polygon mission in server yet...")
                return

            poly = mission["polygon"]

            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for pt in poly:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = (pt["lon"] - poly[0]["lon"]) * 111000 
                pose.pose.position.y = (pt["lat"] - poly[0]["lat"]) * 111000
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0 
                path_msg.poses.append(pose)

            self.publisher_.publish(path_msg)
            self.get_logger().info("Path published from polygon mission")

        except Exception as e:
            self.get_logger().error(f"Error fetching mission: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionToPath()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
