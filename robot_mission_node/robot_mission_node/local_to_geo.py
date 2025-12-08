#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

R = 6378137  # Earth radius (meters)

class LocalToGeo(Node):
    def __init__(self):
        super().__init__("local_to_geo")

        self.origin_lat = None
        self.origin_lon = None

        self.create_subscription(String, '/map_origin', self.origin_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.gps_pub = self.create_publisher(String, '/robot_gps', 10)

    def origin_cb(self, msg):
        data = json.loads(msg.data)
        self.origin_lat = math.radians(data["lat"])
        self.origin_lon = math.radians(data["lon"])
        self.get_logger().info(f"Updated origin → lat={data['lat']}, lon={data['lon']}")

    def odom_cb(self, msg):
        if self.origin_lat is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        dLat = (y / R)
        dLon = (x / (R * math.cos(self.origin_lat)))

        lat = math.degrees(self.origin_lat + dLat)
        lon = math.degrees(self.origin_lon + dLon)

        out = {"lat": lat, "lon": lon}
        
        gps_msg = String()
        gps_msg.data = json.dumps(out)
        self.gps_pub.publish(gps_msg)

def main():
    rclpy.init()
    node = LocalToGeo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
