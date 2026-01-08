#!/usr/bin/env python3
"""
Autonomous Explorer Node

Automatically navigates the boat to explore the entire terrain and build a map.
Uses LIDAR for obstacle detection and avoidance.
Implements a wall-following / frontier exploration strategy.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import time
from enum import Enum
from collections import deque


class ExplorerState(Enum):
    FORWARD = 1
    TURNING_LEFT = 2
    TURNING_RIGHT = 3
    REVERSING = 4
    STOPPED = 5
    RECOVERING = 6


class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        
        # Use simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # Declare parameters - tuned for high angular drag boat
        self.declare_parameter('forward_speed', 30.0)      # Forward thrust (higher for momentum)
        self.declare_parameter('turn_speed', 35.0)         # Higher side thrust for turning
        self.declare_parameter('turn_ratio', 0.3)          # Ratio of slow side during turn (0.3 = 30% of fast side)
        self.declare_parameter('reverse_speed', -15.0)     # Reverse thrust
        self.declare_parameter('min_front_distance', 6.0)  # Start slowing down earlier
        self.declare_parameter('critical_distance', 3.5)   # Must turn/stop - give more room
        self.declare_parameter('side_clearance', 2.5)      # Minimum side clearance
        self.declare_parameter('turn_duration', 3.0)       # Longer turn time for heavy boat
        self.declare_parameter('reverse_duration', 2.0)    # Longer reverse time
        self.declare_parameter('stuck_threshold', 0.3)     # Higher threshold - boat moves slow
        self.declare_parameter('stuck_time', 8.0)          # More time before stuck detection
        
        # Get parameters
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.turn_ratio = self.get_parameter('turn_ratio').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.min_front_distance = self.get_parameter('min_front_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.side_clearance = self.get_parameter('side_clearance').value
        self.turn_duration = self.get_parameter('turn_duration').value
        self.reverse_duration = self.get_parameter('reverse_duration').value
        self.stuck_threshold = self.get_parameter('stuck_threshold').value
        self.stuck_time = self.get_parameter('stuck_time').value
        
        # Publishers for thrust commands
        self.left_thrust_pub = self.create_publisher(Float64, '/boat_1/left_thrust_cmd', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/boat_1/right_thrust_cmd', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/boat_1/odom', self.odom_callback, 10)
        
        # State machine
        self.state = ExplorerState.FORWARD
        self.state_start_time = time.time()
        self.preferred_turn = 'left'  # Alternate turning direction
        
        # Obstacle detection zones (angles in radians)
        # Front: -30° to +30° (0.52 rad)
        # Front-left: +30° to +60°
        # Front-right: -30° to -60°
        # Left: +60° to +120°
        # Right: -60° to -120°
        self.front_min = -0.52
        self.front_max = 0.52
        
        # Distance measurements from LIDAR
        self.front_distance = float('inf')
        self.front_left_distance = float('inf')
        self.front_right_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.rear_distance = float('inf')
        
        # Position tracking for stuck detection
        self.position_history = deque(maxlen=50)
        self.last_position = None
        self.last_position_time = None
        
        # Odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Recovery state
        self.recovery_stage = 0
        self.recovery_start_time = None
        
        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Statistics
        self.total_distance = 0.0
        self.start_time = time.time()
        self.obstacle_avoidances = 0
        
        self.get_logger().info('=== Autonomous Explorer Started ===')
        self.get_logger().info(f'  Forward speed: {self.forward_speed}')
        self.get_logger().info(f'  Min front distance: {self.min_front_distance}m')
        self.get_logger().info(f'  Critical distance: {self.critical_distance}m')
        self.get_logger().info('Press Ctrl+C to stop exploration')

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Process odometry for position tracking."""
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        
        # Calculate distance traveled
        if self.last_position is not None:
            dx = new_x - self.current_x
            dy = new_y - self.current_y
            self.total_distance += math.sqrt(dx*dx + dy*dy)
        
        self.current_x = new_x
        self.current_y = new_y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # Track position for stuck detection
        current_time = time.time()
        self.position_history.append((current_time, new_x, new_y))

    def scan_callback(self, msg):
        """Process LIDAR scan to detect obstacles in different zones."""
        if len(msg.ranges) == 0:
            return
        
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        # Initialize zone distances
        front_readings = []
        front_left_readings = []
        front_right_readings = []
        left_readings = []
        right_readings = []
        rear_readings = []
        
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            
            angle = angle_min + i * angle_increment
            
            # Categorize by zone
            if -0.52 <= angle <= 0.52:  # Front (-30° to +30°)
                front_readings.append(range_val)
            elif 0.52 < angle <= 1.05:  # Front-left (+30° to +60°)
                front_left_readings.append(range_val)
            elif -1.05 <= angle < -0.52:  # Front-right (-60° to -30°)
                front_right_readings.append(range_val)
            elif 1.05 < angle <= 2.09:  # Left (+60° to +120°)
                left_readings.append(range_val)
            elif -2.09 <= angle < -1.05:  # Right (-120° to -60°)
                right_readings.append(range_val)
            elif abs(angle) > 2.6:  # Rear (±150° to ±180°)
                rear_readings.append(range_val)
        
        # Get minimum distances for each zone
        self.front_distance = min(front_readings) if front_readings else float('inf')
        self.front_left_distance = min(front_left_readings) if front_left_readings else float('inf')
        self.front_right_distance = min(front_right_readings) if front_right_readings else float('inf')
        self.left_distance = min(left_readings) if left_readings else float('inf')
        self.right_distance = min(right_readings) if right_readings else float('inf')
        self.rear_distance = min(rear_readings) if rear_readings else float('inf')

    def is_stuck(self):
        """Check if the robot is stuck (hasn't moved significantly)."""
        if len(self.position_history) < 20:
            return False
        
        # Check movement over the last few seconds
        current_time = time.time()
        old_positions = [(t, x, y) for t, x, y in self.position_history 
                         if current_time - t > self.stuck_time * 0.5]
        
        if not old_positions:
            return False
        
        old_t, old_x, old_y = old_positions[0]
        dx = self.current_x - old_x
        dy = self.current_y - old_y
        distance = math.sqrt(dx*dx + dy*dy)
        time_elapsed = current_time - old_t
        
        if time_elapsed > self.stuck_time and distance < self.stuck_threshold:
            return True
        
        return False

    def send_thrust(self, left, right):
        """Send thrust commands to both motors."""
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = float(left)
        right_msg.data = float(right)
        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)

    def stop(self):
        """Stop the boat."""
        self.send_thrust(0.0, 0.0)

    def go_forward(self, speed_factor=1.0):
        """Move forward with optional speed reduction."""
        speed = self.forward_speed * speed_factor
        self.send_thrust(speed, speed)

    def turn_left(self):
        """Turn left while maintaining forward momentum (arc turn).
        Both thrusters positive but differential - works with high angular drag."""
        # Right thruster high, left thruster low (but still positive for forward motion)
        self.send_thrust(self.turn_speed * self.turn_ratio, self.turn_speed)

    def turn_right(self):
        """Turn right while maintaining forward momentum (arc turn).
        Both thrusters positive but differential - works with high angular drag."""
        # Left thruster high, right thruster low (but still positive for forward motion)
        self.send_thrust(self.turn_speed, self.turn_speed * self.turn_ratio)

    def hard_turn_left(self):
        """Hard left turn - slower but tighter."""
        self.send_thrust(0.0, self.turn_speed)

    def hard_turn_right(self):
        """Hard right turn - slower but tighter."""
        self.send_thrust(self.turn_speed, 0.0)

    def reverse(self):
        """Move backward."""
        self.send_thrust(self.reverse_speed, self.reverse_speed)
    
    def reverse_turn_left(self):
        """Reverse while turning left."""
        self.send_thrust(self.reverse_speed * 0.5, self.reverse_speed)
    
    def reverse_turn_right(self):
        """Reverse while turning right."""
        self.send_thrust(self.reverse_speed, self.reverse_speed * 0.5)

    def choose_turn_direction(self):
        """Choose the best direction to turn based on clearance."""
        # Combine front-side and side distances
        left_clearance = min(self.front_left_distance, self.left_distance)
        right_clearance = min(self.front_right_distance, self.right_distance)
        
        # Add some hysteresis - prefer current direction unless other is much better
        if self.preferred_turn == 'left':
            if right_clearance > left_clearance * 1.5:
                return 'right'
            return 'left'
        else:
            if left_clearance > right_clearance * 1.5:
                return 'left'
            return 'right'

    def change_state(self, new_state):
        """Change to a new state."""
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
            self.state = new_state
            self.state_start_time = time.time()

    def control_loop(self):
        """Main control loop - runs at 10Hz."""
        current_time = time.time()
        state_duration = current_time - self.state_start_time
        
        # Check if stuck
        if self.is_stuck() and self.state != ExplorerState.RECOVERING:
            self.get_logger().warn('Robot appears stuck! Entering recovery mode.')
            self.change_state(ExplorerState.RECOVERING)
            self.recovery_stage = 0
            self.recovery_start_time = current_time
            self.obstacle_avoidances += 1
            return
        
        # State machine
        if self.state == ExplorerState.FORWARD:
            self.handle_forward_state()
            
        elif self.state == ExplorerState.TURNING_LEFT:
            if state_duration >= self.turn_duration:
                self.change_state(ExplorerState.FORWARD)
                self.preferred_turn = 'right'  # Alternate next time
            else:
                self.turn_left()
                
        elif self.state == ExplorerState.TURNING_RIGHT:
            if state_duration >= self.turn_duration:
                self.change_state(ExplorerState.FORWARD)
                self.preferred_turn = 'left'  # Alternate next time
            else:
                self.turn_right()
                
        elif self.state == ExplorerState.REVERSING:
            if state_duration >= self.reverse_duration:
                # After reversing, turn away from obstacle
                direction = self.choose_turn_direction()
                if direction == 'left':
                    self.change_state(ExplorerState.TURNING_LEFT)
                else:
                    self.change_state(ExplorerState.TURNING_RIGHT)
            else:
                # Check if rear is clear
                if self.rear_distance < self.critical_distance:
                    # Can't reverse, must turn
                    direction = self.choose_turn_direction()
                    if direction == 'left':
                        self.change_state(ExplorerState.TURNING_LEFT)
                    else:
                        self.change_state(ExplorerState.TURNING_RIGHT)
                else:
                    self.reverse()
                    
        elif self.state == ExplorerState.RECOVERING:
            self.handle_recovery()
            
        elif self.state == ExplorerState.STOPPED:
            self.stop()
        
        # Periodic status update
        if int(current_time) % 10 == 0 and int(current_time * 10) % 10 == 0:
            self.print_status()

    def handle_forward_state(self):
        """Handle the forward movement state with obstacle avoidance."""
        # Check for obstacles ahead
        if self.front_distance < self.critical_distance:
            # Too close! Need to take action
            self.obstacle_avoidances += 1
            self.get_logger().info(f'Obstacle at {self.front_distance:.1f}m - taking evasive action')
            
            # Decide: reverse or turn
            if self.front_distance < self.critical_distance * 0.7:
                # Very close, reverse first
                self.change_state(ExplorerState.REVERSING)
            else:
                # Can just turn
                direction = self.choose_turn_direction()
                if direction == 'left':
                    self.change_state(ExplorerState.TURNING_LEFT)
                else:
                    self.change_state(ExplorerState.TURNING_RIGHT)
                    
        elif self.front_distance < self.min_front_distance:
            # Getting close, slow down and prepare to turn
            speed_factor = (self.front_distance - self.critical_distance) / (self.min_front_distance - self.critical_distance)
            speed_factor = max(0.3, min(1.0, speed_factor))
            
            # Slight steering towards more open side
            if self.front_left_distance > self.front_right_distance * 1.3:
                # Veer left slightly
                self.send_thrust(self.forward_speed * speed_factor * 0.7, 
                               self.forward_speed * speed_factor)
            elif self.front_right_distance > self.front_left_distance * 1.3:
                # Veer right slightly
                self.send_thrust(self.forward_speed * speed_factor, 
                               self.forward_speed * speed_factor * 0.7)
            else:
                self.go_forward(speed_factor)
                
        else:
            # Clear ahead, full speed with slight wandering for exploration
            # Add gentle turns based on which side has more space
            if self.left_distance > self.right_distance * 1.5 and self.left_distance > 10:
                # More space on left, gently explore that way
                self.send_thrust(self.forward_speed * 0.9, self.forward_speed)
            elif self.right_distance > self.left_distance * 1.5 and self.right_distance > 10:
                # More space on right
                self.send_thrust(self.forward_speed, self.forward_speed * 0.9)
            else:
                self.go_forward()

    def handle_recovery(self):
        """Multi-stage recovery from being stuck."""
        current_time = time.time()
        stage_time = current_time - self.recovery_start_time
        
        if self.recovery_stage == 0:
            # Stage 0: Stop
            self.stop()
            if stage_time > 1.0:
                self.recovery_stage = 1
                self.recovery_start_time = current_time
                
        elif self.recovery_stage == 1:
            # Stage 1: Reverse with slight turn
            if self.rear_distance > self.critical_distance:
                # Reverse while turning away from the closer side
                if self.left_distance < self.right_distance:
                    self.reverse_turn_right()
                else:
                    self.reverse_turn_left()
            else:
                self.stop()
            if stage_time > 3.0:
                self.recovery_stage = 2
                self.recovery_start_time = current_time
                
        elif self.recovery_stage == 2:
            # Stage 2: Hard turn towards most open direction
            left_total = self.left_distance + self.front_left_distance
            right_total = self.right_distance + self.front_right_distance
            
            if left_total > right_total:
                self.hard_turn_left()
            else:
                self.hard_turn_right()
                
            if stage_time > 4.0:
                self.recovery_stage = 3
                self.recovery_start_time = current_time
                
        elif self.recovery_stage == 3:
            # Stage 3: Try moving forward slowly
            self.go_forward(0.6)
            if stage_time > 3.0:
                # Reset and return to normal
                self.position_history.clear()
                self.change_state(ExplorerState.FORWARD)
                self.get_logger().info('Recovery complete, resuming exploration')

    def print_status(self):
        """Print current exploration status."""
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'Explorer Status: pos=({self.current_x:.1f}, {self.current_y:.1f}), '
            f'dist={self.total_distance:.1f}m, avoidances={self.obstacle_avoidances}, '
            f'front={self.front_distance:.1f}m, state={self.state.name}'
        )

    def shutdown(self):
        """Clean shutdown."""
        self.stop()
        elapsed = time.time() - self.start_time
        self.get_logger().info('=== Exploration Summary ===')
        self.get_logger().info(f'  Total time: {elapsed:.1f} seconds')
        self.get_logger().info(f'  Distance traveled: {self.total_distance:.1f} meters')
        self.get_logger().info(f'  Obstacle avoidances: {self.obstacle_avoidances}')


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
