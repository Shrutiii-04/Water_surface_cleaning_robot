import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import time
import threading
import websocket
import cv2

WS_URL = "ws://5server_ip_address:8080"
ROBOT_ID = "robot_id_as_in_database"
ROBOT_SECRET = "robot_secret_key"


class RobotWsBridge(Node):
    def __init__(self):
        super().__init__("robot_ws_bridge")
        self.publisher_ = self.create_publisher(String, "/joystick_cmd", 10)
        self.get_logger().info("Robot WS bridge starting...")

        self.ws = None
        self._robot_authenticated = False

        
        self._thread = threading.Thread(target=self._ws_loop, daemon=True)
        self._thread.start()

        
        self._video_thread = threading.Thread(target=self._video_stream_loop, daemon=True)
        self._video_thread.start()

    
    def _ws_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to WS {WS_URL} ...")

                self.ws = websocket.WebSocketApp(
                    WS_URL,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                )

                
                self.ws.run_forever()
            except Exception as e:
                self.get_logger().error(f"WS loop error: {e}")

            self._robot_authenticated = False
            self.get_logger().warn("WS disconnected, retry in 2s...")
            time.sleep(2)

    
    def _on_open(self, ws):
        self.get_logger().info("WS connected, sending auth_robot...")

        auth_msg = {
            "type": "auth_robot",
            "robotId": ROBOT_ID,
            "secretKey": ROBOT_SECRET,
        }
        ws.send(json.dumps(auth_msg))

    
    def _on_message(self, ws, message: str):
        self.get_logger().info(f"WS message: {message}")

        try:
            data = json.loads(message)
        except Exception:
            
            msg = String()
            msg.data = message
            self.publisher_.publish(msg)
            return

        
        if isinstance(data, dict) and data.get("status") == "robot_authenticated":
            self._robot_authenticated = True
            self.get_logger().info("Robot authenticated with WS server")
            return

        
        msg = String()
        msg.data = json.dumps(data)
        self.publisher_.publish(msg)

    def _on_error(self, ws, error):
        self.get_logger().error(f"WS error: {error}")

    def _on_close(self, ws, code, reason):
        self.get_logger().warn(f"WS closed: code={code}, reason={reason}")
        self._robot_authenticated = False


    def _video_stream_loop(self):
        """Continuously capture webcam frames & send through WebSocket."""
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            self.get_logger().error("Cannot open webcam!")
            return

        self.get_logger().info("Video streaming started using laptop webcam")

        while rclpy.ok():
   
            if (
                self.ws is None
                or not getattr(self.ws, "sock", None)
                or not self.ws.sock.connected
                or not self._robot_authenticated
            ):
                time.sleep(0.1)
                continue

            ret, frame = cap.read()
            if not ret:
                continue

            success, jpeg = cv2.imencode(".jpg", frame)
            if not success:
                continue

            try:

                self.ws.send(jpeg.tobytes(), opcode=websocket.ABNF.OPCODE_BINARY)
            except Exception as e:
                self.get_logger().warn(f"Video send failed: {e}")

            time.sleep(0.05)  


def main(args=None):
    rclpy.init(args=args)
    node = RobotWsBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()