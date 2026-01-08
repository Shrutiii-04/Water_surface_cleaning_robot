#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

HELP_MSG = """
╔═══════════════════════════════════════╗
║       BOAT KEYBOARD CONTROL           ║
╠═══════════════════════════════════════╣
║                                       ║
║         W / I  =  Forward             ║
║         S / K  =  Backward            ║
║         A / J  =  Turn Left           ║
║         D / L  =  Turn Right          ║
║         Q      =  Spin Left           ║
║         E      =  Spin Right          ║
║                                       ║
║         SPACE  =  STOP                ║
║         X / Z  =  Quit                ║
║                                       ║
║   1-5  = Set speed (1=slow, 5=fast)   ║
║                                       ║
║   HOLD key to move continuously!      ║
╚═══════════════════════════════════════╝
"""


class BoatTeleop(Node):
    def __init__(self):
        super().__init__('boat_teleop')
        
        # Rear thrusters (forward/backward + turning)
        self.left_pub = self.create_publisher(Float64, '/boat_1/left_thrust_cmd', 10)
        self.right_pub = self.create_publisher(Float64, '/boat_1/right_thrust_cmd', 10)
        
        # Front thrusters (lateral drift correction)
        self.front_left_pub = self.create_publisher(Float64, '/boat_1/front_left_thrust_cmd', 10)
        self.front_right_pub = self.create_publisher(Float64, '/boat_1/front_right_thrust_cmd', 10)
        
        # Speed levels - propeller angular velocities (rad/s)
        # Ignition thruster converts this to thrust
        self.speed_levels = [50.0, 100.0, 200.0, 300.0, 400.0]
        self.speed_idx = 2  # Start at medium
        self.thrust = self.speed_levels[self.speed_idx]
        
        # Current thrust values
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0
        
        # Fast publishing timer
        self.timer = self.create_timer(0.02, self.publish_thrust)  # 50Hz

    def publish_thrust(self):
        l_msg = Float64()
        r_msg = Float64()
        fl_msg = Float64()
        fr_msg = Float64()
        
        l_msg.data = self.left_thrust
        r_msg.data = self.right_thrust
        fl_msg.data = self.front_left_thrust
        fr_msg.data = self.front_right_thrust
        
        self.left_pub.publish(l_msg)
        self.right_pub.publish(r_msg)
        self.front_left_pub.publish(fl_msg)
        self.front_right_pub.publish(fr_msg)

    def forward(self):
        # Left prop: axis +X, needs POSITIVE to push backward
        # Right prop: axis -X, needs NEGATIVE to push backward  
        # Both push boat FORWARD
        self.left_thrust = self.thrust
        self.right_thrust = -self.thrust
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0

    def backward(self):
        # Opposite of forward
        self.left_thrust = -self.thrust
        self.right_thrust = self.thrust
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0

    def turn_left(self):
        # Right side pushes harder, left lighter
        self.left_thrust = self.thrust * 0.2
        self.right_thrust = -self.thrust
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0

    def turn_right(self):
        # Left side pushes harder, right lighter
        self.left_thrust = self.thrust
        self.right_thrust = -self.thrust * 0.2
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0

    def spin_left(self):
        # Left goes backward, right goes forward -> CCW spin
        self.left_thrust = -self.thrust * 0.6
        self.right_thrust = -self.thrust * 0.6
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0

    def spin_right(self):
        # Left goes forward, right goes backward -> CW spin
        self.left_thrust = self.thrust * 0.6
        self.right_thrust = self.thrust * 0.6
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0

    def stop(self):
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.front_left_thrust = 0.0
        self.front_right_thrust = 0.0

    def set_speed(self, level):
        if 1 <= level <= 5:
            self.speed_idx = level - 1
            self.thrust = self.speed_levels[self.speed_idx]
            return True
        return False


def get_key(timeout=0.08):
    """Get key with short timeout for responsive controls"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main():
    rclpy.init()
    node = BoatTeleop()
    
    print(HELP_MSG)
    print(f"Speed: {node.thrust} (level {node.speed_idx + 1})")
    
    key_held = False
    
    try:
        while rclpy.ok():
            key = get_key(0.08)
            
            if key:
                key_held = True
                k = key.lower()
                
                # Movement
                if k in ['w', 'i']:
                    node.forward()
                elif k in ['s', 'k']:
                    node.backward()
                elif k in ['a', 'j']:
                    node.turn_left()
                elif k in ['d', 'l']:
                    node.turn_right()
                elif k == 'q':
                    node.spin_left()
                elif k == 'e':
                    node.spin_right()
                elif k == ' ':
                    node.stop()
                    print("■ STOPPED")
                
                
                elif k in ['1', '2', '3', '4', '5']:
                    if node.set_speed(int(k)):
                        print(f"Speed: {node.thrust} (level {k})")
                
                elif k in ['x', 'z', '\x03', '\x1b']:
                    print("Quitting...")
                    break
            else:
                
                if key_held:
                    node.stop()
                    key_held = False
            
            rclpy.spin_once(node, timeout_sec=0.001)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        node.stop()
        node.publish_thrust()
        node.destroy_node()
        rclpy.shutdown()
        print("Stopped.")


if __name__ == '__main__':
    main()