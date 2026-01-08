import json
import requests
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BACKEND_URL = "http://server_ip_address:3000"
BROKER_IP = "server_ip_address"
ROBOT_ID = "robot_id_as_in_database"  
MQTT_PASSWORD = "robot_password_as_in_database" 

# MQTT Node
class MqttBridge(Node):
    def __init__(self):
        super().__init__("mqtt_bridge")
        self.mqtt_username = None
        self.jwt_token = None
        self.client = None
        self.mission_pub = self.create_publisher(String, "mission_in", 10)
        self.create_timer(1.0, self.publish_telemetry)
        if self.fetch_mqtt_token():
            self.connect_mqtt()
        else:
            self.get_logger().error("Failed to authenticate robot. Exiting.")
            exit(1)

    def fetch_mqtt_token(self):
        self.get_logger().info("Requesting MQTT JWT token from backend...")
        try:
            response = requests.post(
                f"{BACKEND_URL}/robot-mqtt-token",
                json={
                    "robotId": ROBOT_ID,
                    "mqttPassword": MQTT_PASSWORD,
                },
                timeout=5
            )
        except Exception as e:
            self.get_logger().error(f"HTTP ERROR: {e}")
            return False
        data = response.json()
        self.get_logger().info(f"Token response: {data}")
        if "error" in data:
            return False
        self.mqtt_username = data["mqttUsername"]
        self.jwt_token = data["token"]
        return True

    #  MQTT connect
    def connect_mqtt(self):
        self.get_logger().info("Connecting to MQTT broker...")

        self.client = mqtt.Client(client_id=ROBOT_ID)
        self.client.username_pw_set(self.mqtt_username, self.jwt_token)

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        try:
            self.client.connect(BROKER_IP, 1883, 60)
            self.client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT CONNECT ERROR: {e}")

    #  MQTT callbacks
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected successfully!")
            topic = f"robot/{ROBOT_ID}/cmd/mission"
            client.subscribe(topic)
            self.get_logger().info(f"Subscribed to {topic}")
        else:
            self.get_logger().error(f"MQTT failed rc={rc}")

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("MQTT disconnected. Reconnecting...")
        self.fetch_mqtt_token()  
        self.connect_mqtt()   

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        self.get_logger().info(f"Received mission: {payload}")

        ros_msg = String()
        ros_msg.data = payload
        self.mission_pub.publish(ros_msg)

    #  Fake telemetry data
    def publish_telemetry(self):
        if self.client is None:
            return 

        telemetry = {
            "battery": 82,
            "speed": 1.3,
            "distance": 123.0,
            "dustbin": 30,
        }
        topic = f"robot/{ROBOT_ID}/telemetry"
        payload = json.dumps(telemetry)
        self.client.publish(topic, payload)


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    rclpy.spin(node)

    if node.client:
        node.client.loop_stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()