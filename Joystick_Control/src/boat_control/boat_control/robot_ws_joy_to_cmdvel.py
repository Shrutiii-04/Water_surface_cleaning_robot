import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
from collections import deque  

class WsJoyToCmdVel(Node):
    def __init__(self):
        super().__init__('ws_joy_to_cmdvel')

        self.sub = self.create_subscription(String, '/joystick_cmd', self.joy_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    
        self.WINDOW_SIZE = 20  
        
       
        self.x_history = deque(maxlen=self.WINDOW_SIZE)
        self.y_history = deque(maxlen=self.WINDOW_SIZE)

     
        self.LINEAR_SENSITIVITY = 0.1
        self.ANGULAR_SENSITIVITY = 0.1

        self.get_logger().info(f'Smoothing enabled: Averaging last {self.WINDOW_SIZE} inputs')

    def joy_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('type') != 'control': return
            
            payload = data.get('payload', {})
            
        
            raw_y = float(payload.get('x', 0.0)) 
            raw_x = -float(payload.get('y', 0.0)) 

         
            self.x_history.append(raw_x)
            self.y_history.append(raw_y)

          
            avg_x = sum(self.x_history) / len(self.x_history)
            avg_y = sum(self.y_history) / len(self.y_history)

           
            final_x = avg_y * self.LINEAR_SENSITIVITY
            final_z = avg_x * self.ANGULAR_SENSITIVITY

           
            twist = Twist()
            twist.linear.x  = final_x
            twist.angular.z = final_z

            self.pub.publish(twist)
            
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WsJoyToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()