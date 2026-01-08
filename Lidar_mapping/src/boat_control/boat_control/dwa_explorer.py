
"""
DWA Explorer - Dynamic Window Approach with Drift Compensation

Uses the Dynamic Window Approach (DWA) algorithm to:
1. Sample possible velocity commands (left/right thruster combinations)
2. Simulate forward trajectories accounting for boat drift physics
3. Score trajectories based on: goal heading, obstacle clearance, speed
4. Execute the best trajectory while exploring frontiers

Tuned for a heavy boat (2170kg) with:
- High angular drag (5000)
- High angular velocity decay (100.0)
- Differential thrust control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from collections import deque
import time


class DWAExplorer(Node):
    def __init__(self):
        super().__init__('dwa_explorer')
        
        
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        
        
        
        
        
        
        
        
        
        
        
        
        self.max_thrust = 30.0      
        self.min_thrust = -30.0     
        
        
        
        self.boat_mass = 2170.0     
        self.linear_drag = 50.0     
        self.angular_drag = 5000.0  
        self.thrust_to_force = 10.0 
        self.thrust_arm = 0.5       
        
        
        self.max_linear_vel = 2.0   
        self.max_angular_vel = 0.3  
        
        
        
        
        
        
        self.n_linear_samples = 5    
        self.n_angular_samples = 9   
        
        
        self.trajectory_time = 5.0   
        self.trajectory_dt = 0.1     
        
        
        self.goal_weight = 1.0       
        self.obstacle_weight = 8.0   
        self.speed_weight = 0.5      
        self.drift_weight = 1.0      
        
        
        self.obstacle_distance = 8.0   
        self.critical_distance = 3.0   
        self.front_scan_arc = 1.2      
        
        
        
        
        
        self.exploration_radius = 40.0  
        self.goal_tolerance = 3.0       
        self.goal_timeout = 45.0        
        
        
        self.stuck_threshold = 1.0      
        self.stuck_timeout = 15.0       
        
        
        
        
        
        self.left_thrust_pub = self.create_publisher(Float64, '/boat_1/left_thrust_cmd', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/boat_1/right_thrust_cmd', 10)
        self.trajectory_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/boat_1/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        
        
        
        
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0           
        self.vy = 0.0           
        self.omega = 0.0        
        
        
        self.scan_ranges = []
        self.scan_angles = []
        self.scan_msg = None
        
        
        self.map_data = None
        self.map_info = None
        
        
        self.goal_x = None
        self.goal_y = None
        self.goals_visited = []
        self.last_goal_time = time.time()
        
        
        self.pattern_goals = self.generate_exploration_pattern()
        self.pattern_index = 0
        
        
        self.position_history = deque(maxlen=100)
        self.last_significant_move_time = time.time()
        
        
        self.prev_left_thrust = 0.0
        self.prev_right_thrust = 0.0
        
        
        self.start_time = time.time()
        self.distance_traveled = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('=== DWA Explorer Started ===')
        self.get_logger().info('Dynamic Window Approach with Drift Compensation')
        self.get_logger().info(f'  Max thrust: {self.max_thrust}')
        self.get_logger().info(f'  Trajectory lookahead: {self.trajectory_time}s')
        self.get_logger().info('Waiting for sensor data...')

    
    
    
    
    def thrust_to_velocity(self, left_thrust, right_thrust):
        """
        Convert thrust commands to expected linear and angular velocity.
        
        This models the boat's response to differential thrust, accounting for:
        - High angular drag that limits turning rate
        - Velocity decay that limits max speed
        - Thrust asymmetry for turning
        """
        
        avg_thrust = (left_thrust + right_thrust) / 2.0
        
        
        thrust_diff = right_thrust - left_thrust  
        
        
        
        
        target_linear = avg_thrust * self.thrust_to_force / self.linear_drag
        target_linear = np.clip(target_linear, -self.max_linear_vel, self.max_linear_vel)
        
        
        
        
        
        torque = thrust_diff * self.thrust_arm * self.thrust_to_force
        target_angular = torque / self.angular_drag
        target_angular = np.clip(target_angular, -self.max_angular_vel, self.max_angular_vel)
        
        return target_linear, target_angular

    def simulate_trajectory(self, left_thrust, right_thrust, x0, y0, yaw0, vx0, omega0):
        """
        Simulate the boat's trajectory given constant thrust commands.
        
        Returns: List of (x, y, yaw) states along the trajectory
        """
        x, y, yaw = x0, y0, yaw0
        vx, omega = vx0, omega0
        
        trajectory = [(x, y, yaw)]
        
        dt = self.trajectory_dt
        
        
        target_vx, target_omega = self.thrust_to_velocity(left_thrust, right_thrust)
        
        for _ in range(int(self.trajectory_time / dt)):
            
            
            linear_tau = 2.5    
            angular_tau = 3.0   
            
            vx += (target_vx - vx) * dt / linear_tau
            omega += (target_omega - omega) * dt / angular_tau
            
            
            x += vx * math.cos(yaw) * dt
            y += vx * math.sin(yaw) * dt
            yaw += omega * dt
            
            trajectory.append((x, y, yaw))
        
        return trajectory

    
    
    
    
    def generate_velocity_samples(self):
        """
        Generate sampled thrust commands within the dynamic window.
        
        The dynamic window limits commands based on:
        1. Velocity limits (what the boat can achieve)
        2. Acceleration limits (smooth changes)
        3. Obstacle proximity (safety)
        """
        samples = []
        
        
        max_thrust_change = 15.0  
        
        left_min = max(self.min_thrust, self.prev_left_thrust - max_thrust_change)
        left_max = min(self.max_thrust, self.prev_left_thrust + max_thrust_change)
        right_min = max(self.min_thrust, self.prev_right_thrust - max_thrust_change)
        right_max = min(self.max_thrust, self.prev_right_thrust + max_thrust_change)
        
        
        for left in np.linspace(left_min, left_max, self.n_linear_samples):
            for right in np.linspace(right_min, right_max, self.n_angular_samples):
                samples.append((left, right))
        
        
        
        useful = [
            (self.max_thrust * 0.7, self.max_thrust * 0.7),   
            (self.max_thrust * 0.5, self.max_thrust * 0.5),   
            (self.max_thrust * 0.3, self.max_thrust * 0.3),   
            (self.max_thrust * 0.2, self.max_thrust * 0.7),   
            (self.max_thrust * 0.7, self.max_thrust * 0.2),   
            (self.max_thrust * 0.4, self.max_thrust * 0.6),   
            (self.max_thrust * 0.6, self.max_thrust * 0.4),   
            (0.0, self.max_thrust * 0.5),                      
            (self.max_thrust * 0.5, 0.0),                      
            (10.0, 10.0),                                      
            (5.0, 15.0),                                       
            (15.0, 5.0),                                       
        ]
        
        for cmd in useful:
            if cmd not in samples:
                samples.append(cmd)
        
        return samples

    def score_trajectory(self, trajectory, goal_x, goal_y):
        """
        Score a trajectory based on multiple objectives.
        
        Returns: Score (higher is better), or very negative if trajectory hits obstacle
        """
        if not trajectory or goal_x is None:
            return -1000.0
        
        
        end_x, end_y, end_yaw = trajectory[-1]
        
        
        
        
        min_clearance = float('inf')
        
        for x, y, yaw in trajectory:
            clearance = self.get_clearance_at(x, y, yaw)
            min_clearance = min(min_clearance, clearance)
        
        
        
        if min_clearance < 1.0:  
            obstacle_score = -50.0
        elif min_clearance < self.critical_distance:
            obstacle_score = -20.0
        else:
            
            clearance_normalized = min(min_clearance / self.obstacle_distance, 1.0)
            obstacle_score = clearance_normalized * self.obstacle_weight
        
        
        
        
        angle_to_goal = math.atan2(goal_y - end_y, goal_x - end_x)
        angle_error = abs(self.normalize_angle(angle_to_goal - end_yaw))
        
        
        heading_score = (math.pi - angle_error) / math.pi * self.goal_weight
        
        
        
        
        start_x, start_y, _ = trajectory[0]
        
        
        start_dist = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        end_dist = math.sqrt((goal_x - end_x)**2 + (goal_y - end_y)**2)
        
        progress = start_dist - end_dist  
        progress_normalized = np.clip(progress / 5.0, -1.0, 1.0)
        speed_score = progress_normalized * self.speed_weight
        
        
        
        
        
        total_yaw_change = 0.0
        for i in range(1, len(trajectory)):
            yaw_change = abs(self.normalize_angle(trajectory[i][2] - trajectory[i-1][2]))
            total_yaw_change += yaw_change
        
        
        drift_penalty = total_yaw_change / (len(trajectory) * 0.1)  
        drift_score = (1.0 - min(drift_penalty, 1.0)) * self.drift_weight
        
        
        
        
        total_score = obstacle_score + heading_score + speed_score + drift_score
        
        return total_score

    def get_clearance_at(self, x, y, yaw):
        """
        Get the minimum clearance to obstacles from a given pose.
        
        Simplified: Just check current LIDAR readings in the direction
        the robot would be heading at this pose.
        """
        if not self.scan_ranges or self.scan_msg is None:
            return 20.0  
        
        
        dx = x - self.x
        dy = y - self.y
        dist_from_current = math.sqrt(dx*dx + dy*dy)
        
        
        if dist_from_current > 0.1:
            heading_to_query = math.atan2(dy, dx)
            heading_diff = self.normalize_angle(heading_to_query - self.yaw)
        else:
            
            heading_diff = self.normalize_angle(yaw - self.yaw)
        
        
        min_clearance = 50.0  
        found_reading = False
        
        for i, r in enumerate(self.scan_ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < 0.5 or r > 45.0:  
                continue
            
            angle = self.scan_angles[i]
            
            
            angle_diff = abs(self.normalize_angle(angle - heading_diff))
            
            if angle_diff < self.front_scan_arc:
                found_reading = True
                
                effective_clearance = r - dist_from_current
                min_clearance = min(min_clearance, effective_clearance)
        
        
        if not found_reading:
            return 20.0
        
        return max(min_clearance, 0.0)

    def select_best_trajectory(self):
        """
        Run DWA to select the best thrust commands.
        
        Returns: (left_thrust, right_thrust, best_trajectory)
        """
        if self.goal_x is None:
            self.goal_x, self.goal_y = self.select_goal()
            self.last_goal_time = time.time()
        
        
        samples = self.generate_velocity_samples()
        
        best_score = float('-inf')
        best_commands = (0.0, 0.0)
        best_trajectory = None
        all_trajectories = []
        
        for left_thrust, right_thrust in samples:
            
            trajectory = self.simulate_trajectory(
                left_thrust, right_thrust,
                self.x, self.y, self.yaw,
                self.vx, self.omega
            )
            
            
            score = self.score_trajectory(trajectory, self.goal_x, self.goal_y)
            
            all_trajectories.append((trajectory, score))
            
            if score > best_score:
                best_score = score
                best_commands = (left_thrust, right_thrust)
                best_trajectory = trajectory
        
        
        self.publish_trajectories(all_trajectories, best_trajectory)
        
        
        if best_score < -100.0:
            self.get_logger().warn('⚠️ No good trajectory - using fallback: turning to find open space')
            
            left_clear = self.get_min_side_distance('left')
            right_clear = self.get_min_side_distance('right')
            front_clear = self.get_min_front_distance()
            
            if front_clear > 5.0:
                
                return 15.0, 15.0, best_trajectory
            elif left_clear > right_clear:
                
                return 5.0, 20.0, best_trajectory
            else:
                
                return 20.0, 5.0, best_trajectory
        
        return best_commands[0], best_commands[1], best_trajectory

    
    
    
    
    def find_frontiers(self):
        """Find frontier points (boundary between known and unknown space)."""
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
        """Select the next exploration goal based on frontiers."""
        frontiers = self.find_frontiers()
        
        if frontiers:
            best_frontier = None
            best_score = float('inf')
            
            for fx, fy in frontiers:
                dist = math.sqrt((fx - self.x)**2 + (fy - self.y)**2)
                
                
                if dist < self.goal_tolerance or dist > self.exploration_radius:
                    continue
                
                
                too_close_to_visited = False
                for vx, vy in self.goals_visited[-20:]:
                    if math.sqrt((fx - vx)**2 + (fy - vy)**2) < 5.0:
                        too_close_to_visited = True
                        break
                
                if too_close_to_visited:
                    continue
                
                
                angle_to_frontier = math.atan2(fy - self.y, fx - self.x)
                angle_diff = abs(self.normalize_angle(angle_to_frontier - self.yaw))
                
                
                score = dist + angle_diff * 10.0
                
                if score < best_score:
                    best_score = score
                    best_frontier = (fx, fy)
            
            if best_frontier:
                self.get_logger().info(
                    f'🎯 New frontier goal: ({best_frontier[0]:.1f}, {best_frontier[1]:.1f}) '
                    f'dist={math.sqrt((best_frontier[0]-self.x)**2 + (best_frontier[1]-self.y)**2):.1f}m'
                )
                return best_frontier
        
        
        if self.pattern_index < len(self.pattern_goals):
            goal = self.pattern_goals[self.pattern_index]
            self.pattern_index += 1
            self.get_logger().info(f'📍 Pattern goal: ({goal[0]:.1f}, {goal[1]:.1f})')
            return goal
        
        self.pattern_index = 0
        return self.pattern_goals[0] if self.pattern_goals else (10.0, 0.0)

    def generate_exploration_pattern(self):
        """Generate an expanding pattern for systematic exploration."""
        goals = []
        step = 10.0
        max_dist = 40.0
        
        for radius in np.arange(step, max_dist, step):
            for angle in np.linspace(0, 2*math.pi, 8, endpoint=False):
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                goals.append((x, y))
        
        return goals

    
    
    
    
    def control_loop(self):
        """Main control loop - runs at 10Hz."""
        if not self.scan_ranges:
            return
        
        
        if self.goal_x is not None:
            dist_to_goal = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
            
            if dist_to_goal < self.goal_tolerance:
                self.get_logger().info(f'✅ Reached goal at ({self.goal_x:.1f}, {self.goal_y:.1f})')
                self.goals_visited.append((self.goal_x, self.goal_y))
                self.goal_x, self.goal_y = self.select_goal()
                self.last_goal_time = time.time()
            
            
            elif time.time() - self.last_goal_time > self.goal_timeout:
                self.get_logger().warn(f'⏱️ Goal timeout, selecting new goal')
                self.goals_visited.append((self.goal_x, self.goal_y))
                self.goal_x, self.goal_y = self.select_goal()
                self.last_goal_time = time.time()
        
        
        if self.is_stuck():
            self.get_logger().warn('🔄 Stuck detected! Reversing...')
            self.execute_recovery()
            return
        
        
        left_thrust, right_thrust, trajectory = self.select_best_trajectory()
        
        
        min_front = self.get_min_front_distance()
        current_speed = math.sqrt(self.vx**2 + self.vy**2)
        
        
        
        
        stopping_distance = (current_speed ** 2) / (2 * 0.3) + 1.0  
        
        if min_front < self.critical_distance:
            
            self.get_logger().warn(f'🚨 EMERGENCY! Obstacle at {min_front:.1f}m - FULL REVERSE')
            left_thrust = -self.max_thrust
            right_thrust = -self.max_thrust
        elif min_front < self.critical_distance + 2.0:
            
            self.get_logger().warn(f'⚠️ DANGER! Obstacle at {min_front:.1f}m - braking hard')
            
            left_min = self.get_min_side_distance('left')
            right_min = self.get_min_side_distance('right')
            if left_min > right_min:
                left_thrust = -20.0
                right_thrust = -5.0  
            else:
                left_thrust = -5.0
                right_thrust = -20.0  
        elif min_front < stopping_distance + 2.0:
            
            self.get_logger().info(f'⚡ Slowing down - obstacle at {min_front:.1f}m, stop dist={stopping_distance:.1f}m')
            
            left_thrust = min(left_thrust, 10.0)
            right_thrust = min(right_thrust, 10.0)
        
        
        if min_front < self.critical_distance + 2.0:
            alpha = 0.7  
        else:
            alpha = 0.4  
        
        left_thrust = alpha * left_thrust + (1 - alpha) * self.prev_left_thrust
        right_thrust = alpha * right_thrust + (1 - alpha) * self.prev_right_thrust
        
        
        if left_thrust < 5.0 and right_thrust < 5.0 and min_front > self.obstacle_distance:
            self.get_logger().info('🚀 Boosting thrust - front is clear but robot too slow')
            left_thrust = 15.0
            right_thrust = 15.0
        
        
        self.publish_thrust(left_thrust, right_thrust)
        
        
        self.prev_left_thrust = left_thrust
        self.prev_right_thrust = right_thrust
        
        
        if int(time.time()) % 5 == 0:
            self.log_status()

    def get_min_front_distance(self):
        """Get minimum distance to obstacle in front."""
        if not self.scan_ranges:
            return float('inf')
        
        min_dist = float('inf')
        for i, r in enumerate(self.scan_ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            
            angle = self.scan_angles[i]
            if abs(angle) < 0.7:  
                min_dist = min(min_dist, r)
        
        return min_dist

    def get_min_side_distance(self, side):
        """Get minimum distance to obstacle on left or right side."""
        if not self.scan_ranges:
            return float('inf')
        
        min_dist = float('inf')
        for i, r in enumerate(self.scan_ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            
            angle = self.scan_angles[i]
            if side == 'left':
                
                if 0.7 < angle < 2.4:
                    min_dist = min(min_dist, r)
            else:
                
                if -2.4 < angle < -0.7:
                    min_dist = min(min_dist, r)
        
        return min_dist

    def is_stuck(self):
        """Detect if the robot is stuck."""
        if len(self.position_history) < 50:
            return False
        
        oldest = self.position_history[0]
        newest = self.position_history[-1]
        
        
        dist = math.sqrt((newest[1] - oldest[1])**2 + (newest[2] - oldest[2])**2)
        time_diff = newest[0] - oldest[0]
        
        if time_diff > self.stuck_timeout and dist < self.stuck_threshold:
            return True
        
        return False

    def execute_recovery(self):
        """Execute recovery maneuver when stuck."""
        
        self.publish_thrust(-20.0, -15.0)
        self.position_history.clear()
        self.last_significant_move_time = time.time()

    def publish_thrust(self, left, right):
        """Publish thrust commands."""
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = float(left)
        right_msg.data = float(right)
        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)

    def publish_trajectories(self, all_trajectories, best_trajectory):
        """Publish trajectory visualization for RViz."""
        marker_array = MarkerArray()
        
        
        if best_trajectory:
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'best_trajectory'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            for x, y, _ in best_trajectory:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.5
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        
        if self.goal_x is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = 'odom'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'goal'
            goal_marker.id = 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = self.goal_x
            goal_marker.pose.position.y = self.goal_y
            goal_marker.pose.position.z = 1.0
            goal_marker.scale.x = 1.0
            goal_marker.scale.y = 1.0
            goal_marker.scale.z = 1.0
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.5
            goal_marker.color.b = 0.0
            goal_marker.color.a = 0.8
            
            marker_array.markers.append(goal_marker)
        
        self.trajectory_pub.publish(marker_array)

    def log_status(self):
        """Log current exploration status."""
        elapsed = time.time() - self.start_time
        
        goal_dist = 0.0
        if self.goal_x is not None:
            goal_dist = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
        
        self.get_logger().info(
            f'📊 Status: pos=({self.x:.1f}, {self.y:.1f}) '
            f'yaw={math.degrees(self.yaw):.0f}° '
            f'goal_dist={goal_dist:.1f}m '
            f'time={elapsed:.0f}s '
            f'goals_visited={len(self.goals_visited)}'
        )

    
    
    
    
    def odom_callback(self, msg):
        """Update robot position and velocity."""
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
        
        
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.omega = msg.twist.twist.angular.z
        
        
        self.position_history.append((time.time(), self.x, self.y))

    def scan_callback(self, msg):
        """Process LIDAR scan."""
        self.scan_msg = msg
        self.scan_ranges = list(msg.ranges)
        self.scan_angles = [msg.angle_min + i * msg.angle_increment 
                           for i in range(len(msg.ranges))]

    def map_callback(self, msg):
        """Store map for frontier detection."""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    
    
    
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = DWAExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DWA Explorer...')
    finally:
        
        node.publish_thrust(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
