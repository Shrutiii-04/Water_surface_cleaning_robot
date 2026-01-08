#!/usr/bin/env python3
"""
Absolute Position Mapper

This node creates an occupancy grid map using the robot's ABSOLUTE position
from odometry, NOT relative scan matching. This prevents map overlap/drift.

The map is built by:
1. Getting the robot's world position from odometry (synchronized with scan)
2. Transforming each LIDAR point to world coordinates
3. Marking obstacles on a persistent occupancy grid
4. Periodically saving the map to a file
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Header
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import math
import yaml
import os
from datetime import datetime
import threading


class AbsoluteMapper(Node):
    def __init__(self):
        super().__init__('absolute_mapper')
        
        # Use simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # Declare parameters
        self.declare_parameter('resolution', 0.1)  # meters per cell
        self.declare_parameter('map_size', 150.0)  # meters (square map) = 1500x1500 cells
        self.declare_parameter('origin_x', -75.0)  # map origin X (centered)
        self.declare_parameter('origin_y', -75.0)  # map origin Y (centered)
        self.declare_parameter('save_dir', '/home/vedanti/simuletion/maps')
        self.declare_parameter('save_interval', 30.0)  # seconds
        self.declare_parameter('obstacle_threshold', 3)  # hits to mark as obstacle
        self.declare_parameter('free_threshold', 2)  # passes to mark as free
        self.declare_parameter('lidar_offset_x', 0.8)  # lidar offset from base
        self.declare_parameter('lidar_offset_z', 0.8)
        self.declare_parameter('min_range', 2.0)  # ignore readings closer than this (filter boat/water reflections)
        self.declare_parameter('max_range', 25.0)  # ignore readings farther than this (filter max-range/water returns)
        self.declare_parameter('range_jump_threshold', 3.0)  # filter out isolated readings (water reflections)
        
        # Get parameters
        self.resolution = self.get_parameter('resolution').value
        self.map_size = self.get_parameter('map_size').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.save_dir = self.get_parameter('save_dir').value
        self.save_interval = self.get_parameter('save_interval').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.range_jump_threshold = self.get_parameter('range_jump_threshold').value
        
        # Calculate grid dimensions
        self.grid_width = int(self.map_size / self.resolution)
        self.grid_height = int(self.map_size / self.resolution)
        
        # Create hit/miss counters for each cell (for probabilistic update)
        self.hit_count = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        self.miss_count = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        
        # Lock for thread safety
        self.map_lock = threading.Lock()
        
        # Last processed pose for change detection
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.min_move_distance = 0.05  # minimum movement to process scan
        self.min_rotation = 0.02  # minimum rotation to process scan
        
        # Use message_filters for synchronized callbacks
        self.odom_sub = Subscriber(self, Odometry, '/boat_1/odom')
        self.scan_sub = Subscriber(self, LaserScan, '/scan')
        
        # Approximate time synchronizer - allows 500ms slack for simulation
        self.sync = ApproximateTimeSynchronizer(
            [self.odom_sub, self.scan_sub],
            queue_size=30,
            slop=0.5  # 500ms slack for simulation timing differences
        )
        self.sync.registerCallback(self.synced_callback)
        
        # Track sync status for debugging
        self.sync_callback_count = 0
        
        # Publishers - use VOLATILE durability to prevent stale map caching between sessions
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
        map_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.map_metadata_pub = self.create_publisher(MapMetaData, '/map_metadata', map_qos)
        
        # Timers
        self.map_publish_timer = self.create_timer(0.5, self.publish_map)
        self.save_timer = self.create_timer(self.save_interval, self.save_map)
        
        # Create save directory
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.scan_count = 0
        
        self.get_logger().info(f'Absolute Mapper initialized (SYNCHRONIZED)')
        self.get_logger().info(f'  Map size: {self.map_size}m x {self.map_size}m')
        self.get_logger().info(f'  Resolution: {self.resolution}m/cell')
        self.get_logger().info(f'  Grid: {self.grid_width} x {self.grid_height} cells')
        self.get_logger().info(f'  Range filter: {self.min_range}m - {self.max_range}m')
        self.get_logger().info(f'  Save directory: {self.save_dir}')
        
        # Publish empty maps on startup to clear any stale map in RViz
        # Use a timer to publish a few times after subscribers connect
        self.startup_clear_count = 0
        self.startup_timer = self.create_timer(0.1, self.startup_clear_map)
    
    def startup_clear_map(self):
        """Publish empty map several times on startup to ensure RViz gets it."""
        self.startup_clear_count += 1
        self.publish_map()
        if self.startup_clear_count >= 10:
            self.startup_timer.cancel()
            self.get_logger().info('Startup map clearing complete - map is fresh')

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def filter_scan(self, ranges, range_min, range_max):
        """
        Filter LIDAR scan to remove water reflections and noise.
        
        Water reflections typically appear as:
        1. Isolated readings (single spike surrounded by inf/max)
        2. Readings at consistent distance (horizontal water surface)
        3. Readings very close to max range
        
        This filter uses:
        1. Range gating (min/max)
        2. Median filtering to remove isolated spikes
        3. Jump detection to identify discontinuities
        """
        filtered = list(ranges)
        n = len(filtered)
        
        # Window size for neighbor comparison
        window = 5
        
        for i in range(n):
            r = filtered[i]
            
            # Skip already invalid readings
            if math.isinf(r) or math.isnan(r):
                continue
            
            # Basic range filtering
            if r < range_min or r > range_max:
                filtered[i] = float('inf')
                continue
            
            # Too close to sensor max - likely no real return
            if r > range_max * 0.9:
                filtered[i] = float('inf')
                continue
            
            # Check if this is an isolated reading (water spike)
            # Count valid neighbors in the window
            valid_neighbors = 0
            neighbor_ranges = []
            
            for j in range(max(0, i - window), min(n, i + window + 1)):
                if j == i:
                    continue
                nr = ranges[j]
                if not math.isinf(nr) and not math.isnan(nr) and nr < range_max * 0.9:
                    valid_neighbors += 1
                    neighbor_ranges.append(nr)
            
            # If this reading has few valid neighbors, it's likely noise
            if valid_neighbors < window // 2:
                filtered[i] = float('inf')
                continue
            
            # Check for sudden range jumps (this reading is very different from neighbors)
            if neighbor_ranges:
                median_neighbor = sorted(neighbor_ranges)[len(neighbor_ranges) // 2]
                if abs(r - median_neighbor) > self.range_jump_threshold:
                    filtered[i] = float('inf')
                    continue
        
        return filtered

    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid cell indices."""
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def is_valid_cell(self, gx, gy):
        """Check if grid cell is within bounds."""
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    def bresenham_line(self, x0, y0, x1, y1):
        """Generate points along a line using Bresenham's algorithm."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        if dx > dy:
            err = dx / 2
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        
        return points

    def synced_callback(self, odom_msg: Odometry, scan_msg: LaserScan):
        """Process synchronized odometry and scan messages."""
        self.sync_callback_count += 1
        
        # Extract robot pose from odometry
        robot_x = odom_msg.pose.pose.position.x
        robot_y = odom_msg.pose.pose.position.y
        robot_yaw = self.quaternion_to_yaw(odom_msg.pose.pose.orientation)
        
        # Debug logging - first few callbacks
        if self.sync_callback_count <= 5:
            self.get_logger().info(f'Sync callback #{self.sync_callback_count}: robot=({robot_x:.3f}, {robot_y:.3f}), yaw={math.degrees(robot_yaw):.1f}°, scan ranges={len(scan_msg.ranges)}')
        
        # Check if robot has moved enough to warrant processing
        if self.last_x is not None:
            dx = robot_x - self.last_x
            dy = robot_y - self.last_y
            dist = math.sqrt(dx*dx + dy*dy)
            dyaw = abs(robot_yaw - self.last_yaw)
            if dyaw > math.pi:
                dyaw = 2*math.pi - dyaw
            
            if dist < self.min_move_distance and dyaw < self.min_rotation:
                return  # Haven't moved enough
        
        self.last_x = robot_x
        self.last_y = robot_y
        self.last_yaw = robot_yaw
        
        # Calculate LIDAR position in world frame
        lidar_x = robot_x + self.lidar_offset_x * math.cos(robot_yaw)
        lidar_y = robot_y + self.lidar_offset_x * math.sin(robot_yaw)
        
        # Get robot position in grid coordinates
        robot_gx, robot_gy = self.world_to_grid(lidar_x, lidar_y)
        
        if not self.is_valid_cell(robot_gx, robot_gy):
            return
        
        self.scan_count += 1
        if self.scan_count % 50 == 0:
            self.get_logger().info(f'Processed {self.scan_count} scans, robot at ({robot_x:.2f}, {robot_y:.2f}), yaw={math.degrees(robot_yaw):.1f}°')
        
        # Pre-filter LIDAR readings to remove water reflections and noise
        # Water reflections typically appear as isolated spikes or at consistent ranges
        filtered_ranges = self.filter_scan(scan_msg.ranges, scan_msg.range_min, scan_msg.range_max)
        
        # Process each LIDAR ray
        with self.map_lock:
            angle = scan_msg.angle_min
            for i, range_val in enumerate(filtered_ranges):
                # Skip invalid readings (marked as inf by filter)
                if math.isinf(range_val) or math.isnan(range_val):
                    angle += scan_msg.angle_increment
                    continue
                
                # Additional range check
                if range_val < self.min_range or range_val > self.max_range:
                    angle += scan_msg.angle_increment
                    continue
                
                # Calculate world coordinates of the hit point
                # LIDAR angle is relative to robot heading
                world_angle = robot_yaw + angle
                hit_x = lidar_x + range_val * math.cos(world_angle)
                hit_y = lidar_y + range_val * math.sin(world_angle)
                
                # Convert to grid coordinates
                hit_gx, hit_gy = self.world_to_grid(hit_x, hit_y)
                
                # Ray trace from robot to hit point - mark as free
                free_cells = self.bresenham_line(robot_gx, robot_gy, hit_gx, hit_gy)
                for (fx, fy) in free_cells:
                    if self.is_valid_cell(fx, fy):
                        self.miss_count[fy, fx] += 1
                
                # Mark the hit point as obstacle
                if self.is_valid_cell(hit_gx, hit_gy):
                    self.hit_count[hit_gy, hit_gx] += 1
                
                angle += scan_msg.angle_increment

    def generate_occupancy_grid(self):
        """Generate OccupancyGrid message from hit/miss counts."""
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        
        grid.info.resolution = self.resolution
        grid.info.width = self.grid_width
        grid.info.height = self.grid_height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        # Convert hit/miss counts to occupancy values
        data = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        
        # Calculate occupancy probability
        total = self.hit_count + self.miss_count
        
        # Mark cells as free (0) where we have enough passes and few hits
        free_mask = (self.miss_count >= self.free_threshold) & (self.hit_count < self.obstacle_threshold)
        data[free_mask] = 0
        
        # Mark cells as occupied (100) where we have enough hits
        occupied_mask = self.hit_count >= self.obstacle_threshold
        data[occupied_mask] = 100
        
        # Flatten and convert to list
        grid.data = data.flatten().tolist()
        
        return grid

    def publish_map(self):
        """Publish the current map."""
        grid = self.generate_occupancy_grid()
        self.map_pub.publish(grid)
        self.map_metadata_pub.publish(grid.info)

    def save_map(self):
        """Save the map to files (PGM + YAML format for ROS compatibility)."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Generate map data
        data = np.full((self.grid_height, self.grid_width), 205, dtype=np.uint8)  # Unknown = gray
        
        # Free = white (254), Occupied = black (0)
        free_mask = (self.miss_count >= self.free_threshold) & (self.hit_count < self.obstacle_threshold)
        data[free_mask] = 254
        
        occupied_mask = self.hit_count >= self.obstacle_threshold
        data[occupied_mask] = 0
        
        # Flip for image coordinates (y-axis)
        data = np.flipud(data)
        
        # Save PGM file
        pgm_path = os.path.join(self.save_dir, f'map_{timestamp}.pgm')
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.grid_width} {self.grid_height}\n255\n'.encode())
            f.write(data.tobytes())
        
        # Save YAML metadata
        yaml_path = os.path.join(self.save_dir, f'map_{timestamp}.yaml')
        yaml_data = {
            'image': f'map_{timestamp}.pgm',
            'resolution': self.resolution,
            'origin': [self.origin_x, self.origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f)
        
        # Also save a "latest" version
        latest_pgm = os.path.join(self.save_dir, 'latest_map.pgm')
        latest_yaml = os.path.join(self.save_dir, 'latest_map.yaml')
        
        with open(latest_pgm, 'wb') as f:
            f.write(f'P5\n{self.grid_width} {self.grid_height}\n255\n'.encode())
            f.write(data.tobytes())
        
        yaml_data['image'] = 'latest_map.pgm'
        with open(latest_yaml, 'w') as f:
            yaml.dump(yaml_data, f)
        
        # Count obstacles
        num_obstacles = np.sum(occupied_mask)
        num_free = np.sum(free_mask)
        
        self.get_logger().info(f'Map saved: {pgm_path}')
        self.get_logger().info(f'  Obstacle cells: {num_obstacles}')
        self.get_logger().info(f'  Free cells: {num_free}')

    def save_map_on_shutdown(self):
        """Save final map on shutdown."""
        self.get_logger().info('Saving final map before shutdown...')
        self.save_map()


def main(args=None):
    rclpy.init(args=args)
    node = AbsoluteMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map_on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
