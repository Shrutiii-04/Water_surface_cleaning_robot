#!/usr/bin/env python3
import math
import json
from typing import List, Tuple
import numpy as np
from shapely.geometry import Polygon, Point
from shapely.prepared import prep
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.time import Time

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class BCDPlanner(Node):
    """
    Boustrophedon Coverage Decomposition (simple lawnmower) planner.

    This version includes functionality to track and publish the origin of the mission 
    (the starting position) for robot path tracking.
    """

    def __init__(self):
        super().__init__('bcd_planner')

        self.declare_parameter("spacing_m", 0.5)
        self.declare_parameter("max_points", 2000)
        self.declare_parameter("local_frame", "odom")
        self.declare_parameter("min_spacing", 0.1)

        self.spacing_m = float(self.get_parameter("spacing_m").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.local_frame = str(self.get_parameter("local_frame").value)
        self.min_spacing = float(self.get_parameter("min_spacing").value)

        # Coverage path publisher
        self.publisher_ = self.create_publisher(Path, '/cleaning_coverage_path', 10)
        self.origin_pub = self.create_publisher(String, '/map_origin', 10)
        self.subscription = self.create_subscription(
            String,
            'mission_in',
            self.mission_callback,
            10
        )
        self.latest_polygon = None
        self.get_logger().info("BCD Planner node started and waiting for mission...")

    def mission_callback(self, msg: String):
        """
        Called when mission JSON arrives (from MQTT bridge).
        """
        try:
            mission = json.loads(msg.data)
            coords = mission["geometry"]["coordinates"][0]
            if len(coords) < 3:
                self.get_logger().error("Received polygon has fewer than 3 points.")
                return
            self.latest_polygon = [{"lat": float(c[1]), "lon": float(c[0])} for c in coords]
            self.get_logger().info(f"Received polygon with {len(self.latest_polygon)} points")

            # Publish origin for tracking
            origin = {
                "lat": self.latest_polygon[0]["lat"],
                "lon": self.latest_polygon[0]["lon"]
            }
            origin_msg = String()
            origin_msg.data = json.dumps(origin)
            self.origin_pub.publish(origin_msg)
            self.get_logger().info(f"Published /map_origin → {origin}")
            self.generate_and_publish_path()

        except Exception as e:
            self.get_logger().error(f"Error parsing mission: {e}")


    def latlon_to_local(self, polygon: List[dict]) -> Tuple[List[Tuple[float, float]], float]:
        """
        Convert list of dicts [{'lat':..., 'lon':...}, ...] to local meter coordinates
        using equirectangular approximation around mean latitude.

        Returns:
            points_m: list of (x, y) in meters (x = east, y = north)
            mean_lat_rad: mean latitude in radians (useful for reverse transforms)
        """
        lats = [p["lat"] for p in polygon]
        lons = [p["lon"] for p in polygon]
        mean_lat = sum(lats) / len(lats)
        mean_lat_rad = math.radians(mean_lat)
        meters_per_deg_lat = 111_132
        meters_per_deg_lon = 111_320 * math.cos(mean_lat_rad)
        base_lat = lats[0]
        base_lon = lons[0]
        points_m = []
        for lat, lon in zip(lats, lons):
            dy = (lat - base_lat) * meters_per_deg_lat
            dx = (lon - base_lon) * meters_per_deg_lon
            points_m.append((dx, dy))

        return points_m, mean_lat_rad

    def generate_and_publish_path(self):
        if not self.latest_polygon:
            self.get_logger().warn("No polygon mission received yet; nothing to publish.")
            return

        # Convert polygon to local meters
        poly_m, _ = self.latlon_to_local(self.latest_polygon)
        polygon_shape = Polygon(poly_m)

        if not polygon_shape.is_valid or polygon_shape.is_empty:
            self.get_logger().error("Converted polygon is invalid or empty.")
            return

        prepared_poly = prep(polygon_shape)

        # Generate coverage path
        spacing = max(self.spacing_m, self.min_spacing)
        raw_path = self.generate_coverage_path(prepared_poly, polygon_shape.bounds, spacing)

        if not raw_path:
            self.get_logger().warn("Coverage path generation returned no points.")
            return

        if len(raw_path) > self.max_points:
            indices = np.linspace(0, len(raw_path) - 1, self.max_points).astype(int)
            sampled_path = [raw_path[i] for i in indices]
            self.get_logger().info(
                f"Downsampled path from {len(raw_path)} to {len(sampled_path)} points."
            )
        else:
            sampled_path = raw_path

        # Build ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = self.local_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(sampled_path):
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            if i < len(sampled_path) - 1:
                nx, ny = sampled_path[i + 1]
                dx = nx - x
                dy = ny - y
                yaw = math.atan2(dy, dx)
            else:
                if i > 0:
                    px, py = sampled_path[i - 1]
                    yaw = math.atan2(y - py, x - px)
                else:
                    yaw = 0.0

            pose.pose.orientation = yaw_to_quaternion(yaw)
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info(
            f"Published coverage path with {len(path_msg.poses)} waypoints."
        )

    def generate_coverage_path(self, prepared_poly, bounds, spacing):
        min_x, min_y, max_x, max_y = bounds
        width = max_x - min_x
        height = max_y - min_y

        sweep_along_x = width < height

        path = []
        direction = 1

        if sweep_along_x:
            num_lines = max(1, int(math.ceil(height / spacing)))
            ys = np.linspace(min_y, max_y, num_lines + 1)
            for y in ys:
                if direction == 1:
                    xs = np.arange(min_x, max_x + 1e-6, spacing)
                else:
                    xs = np.arange(max_x, min_x - 1e-6, -spacing)

                for x in xs:
                    if prepared_poly.covers(Point(float(x), float(y))):
                        path.append((float(x), float(y)))

                direction *= -1

        else:
            num_lines = max(1, int(math.ceil(width / spacing)))
            xs = np.linspace(min_x, max_x, num_lines + 1)
            for x in xs:
                if direction == 1:
                    ys = np.arange(min_y, max_y + 1e-6, spacing)
                else:
                    ys = np.arange(max_y, min_y - 1e-6, -spacing)

                for y in ys:
                    if prepared_poly.covers(Point(float(x), float(y))):
                        path.append((float(x), float(y)))

                direction *= -1
        if path:
            compacted = [path[0]]
            for p in path[1:]:
                if abs(p[0] - compacted[-1][0]) > 1e-6 or abs(p[1] - compacted[-1][1]) > 1e-6:
                    compacted.append(p)
            path = compacted

        self.get_logger().info(f"Generated raw coverage path with {len(path)} points.")
        return path


def main(args=None):
    rclpy.init(args=args)
    node = BCDPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down BCD Planner.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
