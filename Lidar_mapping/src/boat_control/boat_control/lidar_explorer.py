
"""
LIDAR-based Explorer Node
Uses LIDAR data to autonomously explore and avoid obstacles.
The robot will map out the region while avoiding collisions.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan


class LidarExplorer(Node):
    def __init__(self):
        super().__init__('lidar_explorer')
        
        
        self.left_pub = self.create_publisher(Float64, '/boat_1/left_thrust_cmd', 10)
        self.right_pub = self.create_publisher(Float64, '/boat_1/right_thrust_cmd', 10)
        
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        
        self.forward_thrust = self.declare_parameter('forward_thrust', 50.0).value
        self.turn_thrust = self.declare_parameter('turn_thrust', 40.0).value
        self.obstacle_distance = self.declare_parameter('obstacle_distance', 3.0).value  
        self.critical_distance = self.declare_parameter('critical_distance', 1.5).value  
        
        
        self.current_state = 'forward'
        self.turn_direction = 1  
        self.last_scan = None
        
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('LIDAR Explorer initialized - mapping mode active')
        self.get_logger().info(f'Obstacle detection distance: {self.obstacle_distance}m')
        self.get_logger().info(f'Critical distance: {self.critical_distance}m')

    def scan_callback(self, msg: LaserScan):
        """Process incoming LIDAR scan data."""
        self.last_scan = msg

    def get_sector_min_distance(self, ranges, angle_min, angle_max, scan_angle_min, scan_angle_increment):
        """Get minimum distance in a sector defined by angles (in radians)."""
        if not ranges:
            return float('inf')
        
        
        start_idx = int((angle_min - scan_angle_min) / scan_angle_increment)
        end_idx = int((angle_max - scan_angle_min) / scan_angle_increment)
        
        
        start_idx = max(0, min(start_idx, len(ranges) - 1))
        end_idx = max(0, min(end_idx, len(ranges) - 1))
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
        
        
        sector_ranges = ranges[start_idx:end_idx + 1]
        valid_ranges = [r for r in sector_ranges if r > 0.1 and not math.isinf(r) and not math.isnan(r)]
        
        return min(valid_ranges) if valid_ranges else float('inf')

    def analyze_scan(self):
        """Analyze LIDAR scan and return distances in different sectors."""
        if self.last_scan is None:
            return None
        
        ranges = list(self.last_scan.ranges)
        angle_min = self.last_scan.angle_min
        angle_increment = self.last_scan.angle_increment
        
        
        
        front_min = self.get_sector_min_distance(ranges, -0.52, 0.52, angle_min, angle_increment)
        
        
        front_left_min = self.get_sector_min_distance(ranges, 0.52, 1.57, angle_min, angle_increment)
        
        
        front_right_min = self.get_sector_min_distance(ranges, -1.57, -0.52, angle_min, angle_increment)
        
        
        left_min = self.get_sector_min_distance(ranges, 1.57, 2.62, angle_min, angle_increment)
        
        
        right_min = self.get_sector_min_distance(ranges, -2.62, -1.57, angle_min, angle_increment)
        
        return {
            'front': front_min,
            'front_left': front_left_min,
            'front_right': front_right_min,
            'left': left_min,
            'right': right_min
        }

    def control_loop(self):
        """Main control loop - decides movement based on LIDAR data."""
        sectors = self.analyze_scan()
        
        if sectors is None:
            
            return
        
        front = sectors['front']
        front_left = sectors['front_left']
        front_right = sectors['front_right']
        
        left_cmd = Float64()
        right_cmd = Float64()
        
        
        if front < self.critical_distance:
            
            self.current_state = 'reverse_turn'
            
            if front_left > front_right:
                self.turn_direction = 1  
            else:
                self.turn_direction = -1  
            
            left_cmd.data = -self.turn_thrust * self.turn_direction
            right_cmd.data = self.turn_thrust * self.turn_direction
            self.get_logger().info(f'CRITICAL! Front: {front:.1f}m - Reversing and turning {"left" if self.turn_direction > 0 else "right"}')
            
        elif front < self.obstacle_distance:
            
            self.current_state = 'turning'
            
            if front_left > front_right:
                self.turn_direction = 1  
            else:
                self.turn_direction = -1  
            
            left_cmd.data = -self.turn_thrust * self.turn_direction * 0.5
            right_cmd.data = self.turn_thrust * self.turn_direction * 0.5
            self.get_logger().debug(f'Obstacle at {front:.1f}m - Turning {"left" if self.turn_direction > 0 else "right"}')
            
        elif front_left < self.obstacle_distance * 0.7:
            
            self.current_state = 'veer_right'
            left_cmd.data = self.forward_thrust
            right_cmd.data = self.forward_thrust * 0.5
            self.get_logger().debug(f'Front-left obstacle at {front_left:.1f}m - Veering right')
            
        elif front_right < self.obstacle_distance * 0.7:
            
            self.current_state = 'veer_left'
            left_cmd.data = self.forward_thrust * 0.5
            right_cmd.data = self.forward_thrust
            self.get_logger().debug(f'Front-right obstacle at {front_right:.1f}m - Veering left')
            
        else:
            
            self.current_state = 'forward'
            left_cmd.data = self.forward_thrust
            right_cmd.data = self.forward_thrust
            self.get_logger().debug(f'Path clear - Moving forward (front: {front:.1f}m)')
        
        
        self.left_pub.publish(left_cmd)
        self.right_pub.publish(right_cmd)

    def destroy_node(self):
        """Stop the boat on shutdown."""
        left_cmd = Float64()
        right_cmd = Float64()
        left_cmd.data = 0.0
        right_cmd.data = 0.0
        self.left_pub.publish(left_cmd)
        self.right_pub.publish(right_cmd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
