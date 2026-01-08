
"""
Smart Explorer - Frontier-Based Exploration with Reactive Navigation

Uses LIDAR data to:
1. Identify frontiers (boundaries between known and unknown space)
2. Select the best frontier to explore
3. Navigate toward it while avoiding obstacles

This provides efficient coverage of the entire map.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
import math
import numpy as np
from enum import Enum
from collections import deque
import time


class NavState(Enum):
    EXPLORING = 1       
    AVOIDING = 2        
    ROTATING = 3        
    STUCK_RECOVERY = 4  


class SmartExplorer(Node):
    def __init__(self):
        super().__init__('smart_explorer')
        
        
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        
        self.declare_parameter('max_speed', 35.0)
        self.declare_parameter('min_speed', 15.0)
        self.declare_parameter('turn_gain', 25.0)         
        self.declare_parameter('obstacle_distance', 4.0)  
        self.declare_parameter('critical_distance', 2.0)  
        self.declare_parameter('goal_tolerance', 3.0)     
        self.declare_parameter('exploration_radius', 40.0) 
        
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.turn_gain = self.get_parameter('turn_gain').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        
        
        self.left_thrust_pub = self.create_publisher(Float64, '/boat_1/left_thrust_cmd', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/boat_1/right_thrust_cmd', 10)
        
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/boat_1/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.state = NavState.EXPLORING
        
        
        self.scan_ranges = []
        self.scan_angles = []
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.min_front_dist = float('inf')
        self.min_left_dist = float('inf')
        self.min_right_dist = float('inf')
        
        
        self.goal_x = None
        self.goal_y = None
        self.goals_visited = []
        self.last_goal_time = time.time()
        self.goal_timeout = 30.0  
        
        
        self.map_data = None
        self.map_info = None
        
        
        self.pattern_goals = self.generate_exploration_pattern()
        self.pattern_index = 0
        
        
        self.position_history = deque(maxlen=100)
        self.last_significant_move_time = time.time()
        self.stuck_threshold = 1.0  
        self.stuck_timeout = 10.0   
        
        
        self.state_start_time = time.time()
        self.rotation_direction = 1  
        
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        
        self.start_time = time.time()
        self.distance_traveled = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.get_logger().info('=== Smart Explorer Started ===')
        self.get_logger().info(f'  Max speed: {self.max_speed}')
        self.get_logger().info(f'  Obstacle distance: {self.obstacle_distance}m')
        self.get_logger().info('Waiting for LIDAR and odometry data...')

    def generate_exploration_pattern(self):
        """Generate a lawnmower/spiral pattern for systematic exploration."""
        goals = []
        
        step = 8.0  
        max_dist = 35.0
        
        
        for radius in np.arange(step, max_dist, step):
            
            goals.append((radius, 0))
            goals.append((radius, radius))
            goals.append((0, radius))
            goals.append((-radius, radius))
            goals.append((-radius, 0))
            goals.append((-radius, -radius))
            goals.append((0, -radius))
            goals.append((radius, -radius))
        
        return goals

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Update robot position."""
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        
        
        dx = new_x - self.last_x
        dy = new_y - self.last_y
        self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        self.last_x = new_x
        self.last_y = new_y
        
        self.x = new_x
        self.y = new_y
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        
        self.position_history.append((time.time(), self.x, self.y))

    def scan_callback(self, msg):
        """Process LIDAR scan."""
        self.scan_ranges = list(msg.ranges)
        self.scan_angles = [msg.angle_min + i * msg.angle_increment 
                           for i in range(len(msg.ranges))]
        
        
        front_ranges = []
        left_ranges = []
        right_ranges = []
        
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            
            angle = self.scan_angles[i]
            
            
            if -0.785 <= angle <= 0.785:
                front_ranges.append(r)
            
            elif 0.785 < angle <= 2.356:
                left_ranges.append(r)
            
            elif -2.356 <= angle < -0.785:
                right_ranges.append(r)
        
        self.min_front_dist = min(front_ranges) if front_ranges else float('inf')
        self.min_left_dist = min(left_ranges) if left_ranges else float('inf')
        self.min_right_dist = min(right_ranges) if right_ranges else float('inf')
        
        self.front_clear = self.min_front_dist > self.obstacle_distance
        self.left_clear = self.min_left_dist > self.critical_distance
        self.right_clear = self.min_right_dist > self.critical_distance

    def map_callback(self, msg):
        """Store map for frontier detection."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def find_frontiers(self):
        """Find frontier points (boundary between known free space and unknown)."""
        if self.map_data is None or self.map_info is None:
            return []
        
        frontiers = []
        height, width = self.map_data.shape
        
        
        step = 5
        
        for y in range(step, height - step, step):
            for x in range(step, width - step, step):
                
                if self.map_data[y, x] != 0:
                    continue
                
                
                has_unknown_neighbor = False
                for dy in [-step, 0, step]:
                    for dx in [-step, 0, step]:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            if self.map_data[ny, nx] == -1:
                                has_unknown_neighbor = True
                                break
                    if has_unknown_neighbor:
                        break
                
                if has_unknown_neighbor:
                    
                    world_x = self.map_info.origin.position.x + x * self.map_info.resolution
                    world_y = self.map_info.origin.position.y + y * self.map_info.resolution
                    frontiers.append((world_x, world_y))
        
        return frontiers

    def select_goal(self):
        """Select the next exploration goal."""
        
        frontiers = self.find_frontiers()
        
        if frontiers:
            
            best_frontier = None
            best_score = float('inf')
            
            for fx, fy in frontiers:
                dist = math.sqrt((fx - self.x)**2 + (fy - self.y)**2)
                
                
                if dist < self.goal_tolerance or dist > self.exploration_radius:
                    continue
                
                
                too_close_to_visited = False
                for vx, vy in self.goals_visited[-10:]:  
                    if math.sqrt((fx - vx)**2 + (fy - vy)**2) < 5.0:
                        too_close_to_visited = True
                        break
                
                if too_close_to_visited:
                    continue
                
                
                angle_to_frontier = math.atan2(fy - self.y, fx - self.x)
                angle_diff = abs(self.normalize_angle(angle_to_frontier - self.yaw))
                
                score = dist + angle_diff * 5.0  
                
                if score < best_score:
                    best_score = score
                    best_frontier = (fx, fy)
            
            if best_frontier:
                self.get_logger().info(f'New frontier goal: ({best_frontier[0]:.1f}, {best_frontier[1]:.1f})')
                return best_frontier
        
        
        if self.pattern_index < len(self.pattern_goals):
            goal = self.pattern_goals[self.pattern_index]
            self.pattern_index += 1
            self.get_logger().info(f'Pattern goal: ({goal[0]:.1f}, {goal[1]:.1f})')
            return goal
        
        
        self.pattern_index = 0
        return self.pattern_goals[0] if self.pattern_goals else (10.0, 0.0)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_stuck(self):
        """Check if robot is stuck."""
        if len(self.position_history) < 50:
            return False
        
        current_time = time.time()
        old_positions = [(t, x, y) for t, x, y in self.position_history 
                        if current_time - t > 5.0]
        
        if not old_positions:
            return False
        
        old_t, old_x, old_y = old_positions[0]
        dist = math.sqrt((self.x - old_x)**2 + (self.y - old_y)**2)
        time_elapsed = current_time - old_t
        
        if time_elapsed > self.stuck_timeout and dist < self.stuck_threshold:
            return True
        
        return False

    def send_thrust(self, left, right):
        """Send thrust commands."""
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = float(left)
        right_msg.data = float(right)
        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)

    def control_loop(self):
        """Main control loop."""
        current_time = time.time()
        
        
        if self.is_stuck() and self.state != NavState.STUCK_RECOVERY:
            self.get_logger().warn('Robot stuck! Entering recovery...')
            self.state = NavState.STUCK_RECOVERY
            self.state_start_time = current_time
            self.rotation_direction = 1 if self.min_left_dist > self.min_right_dist else -1
        
        
        if self.goal_x is None or self.goal_y is None:
            goal = self.select_goal()
            self.goal_x, self.goal_y = goal
            self.last_goal_time = current_time
        
        
        dist_to_goal = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f'Goal reached! Distance traveled: {self.distance_traveled:.1f}m')
            self.goals_visited.append((self.goal_x, self.goal_y))
            goal = self.select_goal()
            self.goal_x, self.goal_y = goal
            self.last_goal_time = current_time
        
        
        if current_time - self.last_goal_time > self.goal_timeout:
            self.get_logger().info('Goal timeout, selecting new goal')
            self.goals_visited.append((self.goal_x, self.goal_y))
            goal = self.select_goal()
            self.goal_x, self.goal_y = goal
            self.last_goal_time = current_time
        
        
        if self.state == NavState.EXPLORING:
            self.do_exploring()
        elif self.state == NavState.AVOIDING:
            self.do_avoiding()
        elif self.state == NavState.ROTATING:
            self.do_rotating()
        elif self.state == NavState.STUCK_RECOVERY:
            self.do_stuck_recovery()
        
        
        if int(current_time) % 15 == 0 and int(current_time * 10) % 10 == 0:
            self.get_logger().info(
                f'Status: pos=({self.x:.1f},{self.y:.1f}), goal=({self.goal_x:.1f},{self.goal_y:.1f}), '
                f'front={self.min_front_dist:.1f}m, traveled={self.distance_traveled:.1f}m'
            )

    def do_exploring(self):
        """Navigate toward goal while avoiding obstacles."""
        
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_error = self.normalize_angle(angle_to_goal - self.yaw)
        
        
        if self.min_front_dist < self.critical_distance:
            
            self.state = NavState.AVOIDING
            self.state_start_time = time.time()
            self.rotation_direction = 1 if self.min_left_dist > self.min_right_dist else -1
            return
        
        if self.min_front_dist < self.obstacle_distance:
            
            
            if self.min_left_dist > self.min_right_dist:
                
                angle_error = max(angle_error, 0.5)
            else:
                
                angle_error = min(angle_error, -0.5)
        
        
        obstacle_factor = min(1.0, (self.min_front_dist - self.critical_distance) / 
                             (self.obstacle_distance - self.critical_distance))
        angle_factor = 1.0 - min(1.0, abs(angle_error) / 1.5)
        
        speed = self.min_speed + (self.max_speed - self.min_speed) * obstacle_factor * angle_factor
        
        
        
        turn_component = self.turn_gain * angle_error
        
        left_thrust = speed - turn_component
        right_thrust = speed + turn_component
        
        
        left_thrust = max(5.0, min(self.max_speed * 1.2, left_thrust))
        right_thrust = max(5.0, min(self.max_speed * 1.2, right_thrust))
        
        self.send_thrust(left_thrust, right_thrust)

    def do_avoiding(self):
        """Obstacle avoidance maneuver."""
        state_time = time.time() - self.state_start_time
        
        if state_time > 3.0:
            
            if self.front_clear:
                self.state = NavState.EXPLORING
            else:
                self.state = NavState.ROTATING
                self.state_start_time = time.time()
            return
        
        
        if self.rotation_direction > 0:
            
            self.send_thrust(10.0, self.turn_gain)
        else:
            
            self.send_thrust(self.turn_gain, 10.0)

    def do_rotating(self):
        """Rotate in place to find clear path."""
        state_time = time.time() - self.state_start_time
        
        if self.front_clear and state_time > 1.0:
            self.state = NavState.EXPLORING
            return
        
        if state_time > 8.0:
            
            self.state = NavState.STUCK_RECOVERY
            self.state_start_time = time.time()
            return
        
        
        if self.rotation_direction > 0:
            self.send_thrust(0.0, self.turn_gain)
        else:
            self.send_thrust(self.turn_gain, 0.0)

    def do_stuck_recovery(self):
        """Multi-stage stuck recovery."""
        state_time = time.time() - self.state_start_time
        
        if state_time < 2.0:
            
            self.send_thrust(0.0, 0.0)
        elif state_time < 5.0:
            
            self.send_thrust(-20.0, -20.0)
        elif state_time < 9.0:
            
            if self.rotation_direction > 0:
                self.send_thrust(0.0, self.turn_gain)
            else:
                self.send_thrust(self.turn_gain, 0.0)
        else:
            
            self.position_history.clear()
            self.goals_visited.append((self.goal_x, self.goal_y))
            goal = self.select_goal()
            self.goal_x, self.goal_y = goal
            self.last_goal_time = time.time()
            self.state = NavState.EXPLORING
            self.get_logger().info('Recovery complete')

    def shutdown(self):
        """Clean shutdown."""
        self.send_thrust(0.0, 0.0)
        elapsed = time.time() - self.start_time
        self.get_logger().info('=== Exploration Summary ===')
        self.get_logger().info(f'  Total time: {elapsed:.1f}s')
        self.get_logger().info(f'  Distance traveled: {self.distance_traveled:.1f}m')
        self.get_logger().info(f'  Goals visited: {len(self.goals_visited)}')


def main(args=None):
    rclpy.init(args=args)
    node = SmartExplorer()
    
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
