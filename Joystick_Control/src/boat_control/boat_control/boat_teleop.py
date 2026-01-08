import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty

msg = """
---------------------------
   DUAL THRUSTER CONTROL
---------------------------
   LEFT MOTOR    |    RIGHT MOTOR
   w : Forward   |    e : Forward
   s : Backward  |    d : Backward

   x : STOP ALL
   q : Quit

   (Press keys to set thrust state)
---------------------------
"""

# CONFIGURATION
# Adjust these if your boat moves backwards when it should go forward
FORWARD_FORCE = -5000.0   # Negative because props push water back
BACKWARD_FORCE = 5000.0
STOP = 0.0

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Simple version without select (works better on some systems)
def get_key_simple():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class BoatTeleop(Node):
    def __init__(self):
        super().__init__('boat_teleop')
        self.left_pub = self.create_publisher(Float64, '/model/boat_1/joint/prop_joint_left/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/model/boat_1/joint/prop_joint_right/cmd_thrust', 10)

    def send_thrust(self, l_val, r_val):
        l_msg = Float64()
        r_msg = Float64()
        l_msg.data = float(l_val)
        r_msg.data = float(r_val)
        self.left_pub.publish(l_msg)
        self.right_pub.publish(r_msg)

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = BoatTeleop()
    
    # Initial state (Stopped)
    left_thrust = STOP
    right_thrust = STOP
    
    print(msg)
    
    try:
        while True:
            key = get_key_simple()
            
            # --- LEFT MOTOR CONTROLS ---
            if key == 'w':
                left_thrust = BACKWARD_FORCE
                print(f"Left: FWD  | Right: {right_thrust}")
            elif key == 's':
                left_thrust = FORWARD_FORCE
                print(f"Left: BCK  | Right: {right_thrust}")

            # --- RIGHT MOTOR CONTROLS ---
            elif key == 'e':
                right_thrust = BACKWARD_FORCE
                print(f"Left: {left_thrust}  | Right: FWD")
            elif key == 'd':
                right_thrust =  FORWARD_FORCE
                print(f"Left: {left_thrust}  | Right: BCK")

            # --- SAFETY CONTROLS ---
            elif key == 'x':
                left_thrust = STOP
                right_thrust = STOP
                print("!!! STOPPED !!!")
            
            elif key == 'q' or key == '\x03': # q or Ctrl-C
                break

            # Send the updated commands
            node.send_thrust(left_thrust, right_thrust)

    except Exception as e:
        print(e)

    finally:
        node.send_thrust(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import select
    main()
