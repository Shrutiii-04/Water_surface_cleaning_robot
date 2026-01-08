import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToThrusters(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_thrusters')

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        self.pub_left = self.create_publisher(Float64, '/boat_1/left_thrust_cmd', 10)
        self.pub_right = self.create_publisher(Float64, '/boat_1/right_thrust_cmd', 10)

        self.gain = 1000.0 
        
        self.last_msg_time = self.get_clock().now()
        self.create_timer(0.1, self.failsafe_check)
        
        self.get_logger().info('Drift Correction Mode Active')

    def cmd_callback(self, msg):
        self.last_msg_time = self.get_clock().now()

        fwd = msg.linear.x 
        turn = msg.angular.z 

        
        drift_factor = -0.1 

        
        if fwd != 0:
            correction = fwd * drift_factor
        else:
            correction = 0.0

        
        corrected_turn = turn - correction

       
        left_power = (fwd + corrected_turn) * self.gain
        right_power = (fwd - corrected_turn) * self.gain

        self.send_to_motors(left_power, right_power)
    

    def failsafe_check(self):
        time_since_last_msg = self.get_clock().now() - self.last_msg_time
        if time_since_last_msg.nanoseconds / 1e9 > 0.5:
            self.send_to_motors(0.0, 0.0)

    def send_to_motors(self, left, right):
        l_msg = Float64()
        r_msg = Float64()
        l_msg.data = float(left)
        r_msg.data = float(right)
        self.pub_left.publish(l_msg)
        self.pub_right.publish(r_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()