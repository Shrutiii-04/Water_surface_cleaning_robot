#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json

class RobotLocationMQTT(Node):
    def __init__(self):
        super().__init__('robot_location_mqtt')

        self.robot_id = "robot_003"

        self.create_subscription(String, '/robot_gps', self.gps_cb, 10)

        self.client = mqtt.Client()
        try:
            # Connect to MQTT broker
            self.client.connect("51.21.101.122", 3000)
            self.client.loop_start()
            self.get_logger().info("MQTT Connected")
        except Exception as e:
            self.get_logger().error(f"MQTT ERROR: {e}")

    def gps_cb(self, msg):
        # Forward GPS payload to MQTT topic
        payload = msg.data
        topic = f"robot/{self.robot_id}/location"
        try:
            self.client.publish(topic, payload)
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

def main():
    rclpy.init()
    node = RobotLocationMQTT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
