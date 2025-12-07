# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # from nav_msgs.msg import Path
# # from geometry_msgs.msg import PoseStamped
# # import requests
# # import math
# # import numpy as np

# # API_ENDPOINT = "http://127.0.0.1:5000/get_mission"

# # class BCDPlanner(Node):
# #     def __init__(self):
# #         super().__init__('bcd_planner')
# #         self.publisher_ = self.create_publisher(Path, '/cleaning_coverage_path', 10)
# #         self.timer = self.create_timer(2.0, self.fetch_and_publish_path)
# #         self.get_logger().info("BCD Planner node started ✅")

# #     def fetch_and_publish_path(self):
# #         try:
# #             response = requests.get(API_ENDPOINT, timeout=2)
# #             mission = response.json().get("data", {})

# #             if not mission or "polygon" not in mission:
# #                 self.get_logger().warn("No polygon mission yet...")
# #                 return

# #             poly = mission["polygon"]
# #             # Convert lat/lon to local meters (simple approximation)
# #             base_lat = poly[0]["lat"]
# #             base_lon = poly[0]["lon"]
# #             polygon_points = []
# #             for pt in poly:
# #                 x = (pt["lon"] - base_lon) * 111000  # longitude -> meters
# #                 y = (pt["lat"] - base_lat) * 111000  # latitude -> meters
# #                 polygon_points.append((x, y))

# #             # Generate coverage path using BCD (lawnmower sweep)
# #             coverage_path = self.generate_coverage_path(polygon_points, spacing=0.5)

# #             # Convert to ROS Path message
# #             path_msg = Path()
# #             path_msg.header.frame_id = "odom"
# #             path_msg.header.stamp = self.get_clock().now().to_msg()
# #             for x, y in coverage_path:
# #                 pose = PoseStamped()
# #                 pose.header = path_msg.header
# #                 pose.pose.position.x = x
# #                 pose.pose.position.y = y
# #                 pose.pose.position.z = 0.0
# #                 pose.pose.orientation.w = 1.0
# #                 path_msg.poses.append(pose)

# #             self.publisher_.publish(path_msg)
# #             self.get_logger().info(f"🧭 Coverage path published with {len(coverage_path)} points ✅")

# #         except Exception as e:
# #             self.get_logger().error(f"Error fetching mission: {e}")

# #     # ------------------ BCD coverage generation ------------------
# #     # def generate_coverage_path(self, polygon, spacing=0.5):
# #     #     """
# #     #     Simple lawnmower path inside bounding box of polygon.
# #     #     polygon: list of (x,y) points
# #     #     spacing: distance between sweep lines
# #     #     """
# #     #     polygon = np.array(polygon)
# #     #     min_x, min_y = polygon.min(axis=0)
# #     #     max_x, max_y = polygon.max(axis=0)

# #     #     coverage_path = []
# #     #     y = min_y
# #     #     direction = 1  # left->right or right->left
# #     #     while y <= max_y:
# #     #         if direction == 1:
# #     #             line_points = [(x, y) for x in np.arange(min_x, max_x, spacing)]
# #     #         else:
# #     #             line_points = [(x, y) for x in np.arange(max_x, min_x, -spacing)]
# #     #         # Keep points inside polygon using simple point-in-polygon test
# #     #         for pt in line_points:
# #     #             if self.point_in_polygon(pt, polygon):
# #     #                 coverage_path.append(pt)
# #     #         y += spacing
# #     #         direction *= -1
# #     #     return coverage_path

# #     def generate_coverage_path(self, polygon, spacing=0.3):
# #         polygon = np.array(polygon)

# #         # Center polygon for visualization clarity
# #         centroid = polygon.mean(axis=0)
# #         polygon = polygon - centroid

# #         min_x, min_y = polygon.min(axis=0)
# #         max_x, max_y = polygon.max(axis=0)

# #         coverage = []
# #         y = min_y
# #         left_to_right = True

# #         while y <= max_y:
# #             if left_to_right:
# #                 xs = np.arange(min_x, max_x, 0.05)  # dense X sweep
# #             else:
# #                 xs = np.arange(max_x, min_x, -0.05)

# #             for x in xs:
# #                 pt = (x, y)
# #                 if self.point_in_polygon(pt, polygon):
# #                     coverage.append(pt)

# #             y += spacing
# #             left_to_right = not left_to_right

# #         return coverage


# #     def point_in_polygon(self, point, polygon):
# #         # Ray casting algorithm for point-in-polygon
# #         x, y = point
# #         n = len(polygon)
# #         inside = False
# #         px, py = polygon[0]
# #         for i in range(1, n + 1):
# #             cx, cy = polygon[i % n]
# #             if ((cy > y) != (py > y)) and \
# #                     (x < (px - cx) * (y - cy) / (py - cy + 1e-6) + cx):
# #                 inside = not inside
# #             px, py = cx, cy
# #         return inside

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = BCDPlanner()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == "__main__":
# #     main()

# # !/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import requests
# import math
# import numpy as np

# # --- New Import ---
# from shapely.geometry import Polygon, Point

# API_ENDPOINT = "http://127.0.0.1:5000/get_mission"

# class BCDPlanner(Node):
#     def __init__(self):
#         super().__init__('bcd_planner')
#         self.publisher_ = self.create_publisher(Path, '/cleaning_coverage_path', 10)
#         self.timer = self.create_timer(2.0, self.fetch_and_publish_path)
#         self.get_logger().info("BCD Planner node started ")

#     def fetch_and_publish_path(self):
#         try:
#             response = requests.get(API_ENDPOINT, timeout=2)
#             mission = response.json().get("data", {})

#             if not mission or "polygon" not in mission:
#                 self.get_logger().warn("No polygon mission yet...")
#                 return

#             poly = mission["polygon"]
#             base_lat = poly[0]["lat"]
#             base_lon = poly[0]["lon"]
#             polygon_points_meters = []
#             for pt in poly:
#                 x = (pt["lon"] - base_lon) * 111000  # longitude -> meters
#                 y = (pt["lat"] - base_lat) * 111000  # latitude -> meters
#                 polygon_points_meters.append((x, y))

#             # Generate coverage path using the robust sweep
#             coverage_path = self.generate_coverage_path(polygon_points_meters, spacing=0.5)

#             # Convert to ROS Path message
#             path_msg = Path()
#             path_msg.header.frame_id = "odom"
#             path_msg.header.stamp = self.get_clock().now().to_msg()
#             for x, y in coverage_path:
#                 pose = PoseStamped()
#                 pose.header = path_msg.header
#                 pose.pose.position.x = x
#                 pose.pose.position.y = y
#                 pose.pose.position.z = 0.0
#                 pose.pose.orientation.w = 1.0
#                 path_msg.poses.append(pose)

#             self.publisher_.publish(path_msg)
#             self.get_logger().info(f"Coverage path published with {len(coverage_path)} points")

#         except Exception as e:
#             self.get_logger().error(f"Error fetching mission: {e}")

#     # ------------------ BCD coverage generation (Refactored) ------------------

#     def generate_coverage_path(self, points_list, spacing=0.5):
#         """
#         Simple lawnmower path inside polygon using Shapely for robust point-in-polygon test.
#         points_list: list of (x,y) points in meters
#         spacing: distance between sweep lines
#         """
#         # Create a robust Shapely polygon object
#         polygon_shape = Polygon(points_list)
        
#         # Get bounding box coordinates
#         min_x, min_y, max_x, max_y = polygon_shape.bounds

#         coverage_path = []
#         y = min_y
#         direction = 1  # left->right (1) or right->left (-1)

#         while y <= max_y:
#             # Generate points along the horizontal line
#             if direction == 1:
#                 xs = np.arange(min_x, max_x, 0.5)
#             else:
#                 xs = np.arange(max_x, min_x, -0.5)
            
#             for x in xs:
#                 # Use Shapely's robust method instead of ray casting
#                 if polygon_shape.contains(Point(x, y)):
#                     coverage_path.append((x, y))

#             y += spacing
#             direction *= -1
            
#         return coverage_path

# # Note: The original point_in_polygon function is removed as it's no longer needed.

# def main(args=None):
#     rclpy.init(args=args)
#     node = BCDPlanner()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # from nav_msgs.msg import Path
# # from geometry_msgs.msg import PoseStamped
# # import requests
# # import numpy as np
# # from shapely.geometry import Polygon, Point, box, LineString, MultiPolygon
# # from shapely.ops import unary_union

# # API_ENDPOINT = "http://127.0.0.1:5000/get_mission"

# # class BCDPlanner(Node):
# #     def __init__(self):
# #         super().__init__('bcd_planner_bcd_real')
# #         self.publisher_ = self.create_publisher(Path, '/cleaning_coverage_path', 10)

# #         # only plan once: trigger fetch_and_publish_path once then stop
# #         self.timer = self.create_timer(1.0, self.fetch_and_publish_path)
# #         self.get_logger().info("BCD (real) Planner node started")

# #         # parameters you can tune
# #         self.sweep_spacing = 0.5     # meters between adjacent sweep lines inside each cell
# #         self.sample_spacing = 0.25   # spacing of points along sweep segments (controls path resolution)
# #         self.publish_once = True

# #     def fetch_and_publish_path(self):
# #         # We'll plan once and stop timer afterwards
# #         try:
# #             response = requests.get(API_ENDPOINT, timeout=2)
# #             mission = response.json().get("data", {})

# #             if not mission or "polygon" not in mission:
# #                 self.get_logger().warn("No polygon mission yet...")
# #                 return

# #             # Convert lat/lon to local meters using first vertex as origin
# #             poly = mission["polygon"]
# #             base_lat = poly[0]["lat"]
# #             base_lon = poly[0]["lon"]
# #             pts_m = [((p["lon"] - base_lon) * 111000, (p["lat"] - base_lat) * 111000) for p in poly]

# #             # Build Shapely polygon (handles concave shapes). If polygon has repeated last point, shapely handles it.
# #             polygon_shape = Polygon(pts_m)
# #             if not polygon_shape.is_valid:
# #                 polygon_shape = polygon_shape.buffer(0)  # attempt fix
# #             if polygon_shape.is_empty:
# #                 self.get_logger().error("Polygon is empty after conversion/fix.")
# #                 return

# #             # 1) Decompose into vertical strips between unique vertex y-values,
# #             #    intersect with polygon -> get many small pieces
# #             pieces = self._slice_polygon_into_strips(polygon_shape)

# #             # 2) Merge touching pieces into connected cells (graph connected components)
# #             cells = self._merge_pieces_into_cells(pieces)

# #             if not cells:
# #                 self.get_logger().error("No cells found in polygon!")
# #                 return

# #             # 3) For each cell generate a lawnmower (boustrophedon) sweep inside that cell
# #             cell_paths = []
# #             for cell in cells:
# #                 sweep = self._generate_lawnmower_for_cell(cell, spacing=self.sweep_spacing, sample_step=self.sample_spacing)
# #                 if sweep:
# #                     cell_paths.append(sweep)

# #             # 4) Stitch cell paths into a single continuous path (simple nearest-neighbor ordering)
# #             continuous_pts = self._stitch_cell_paths(cell_paths)

# #             # 5) Publish as nav_msgs/Path
# #             path_msg = Path()
# #             path_msg.header.frame_id = "odom"
# #             path_msg.header.stamp = self.get_clock().now().to_msg()
# #             for x, y in continuous_pts:
# #                 pose = PoseStamped()
# #                 pose.header = path_msg.header
# #                 pose.pose.position.x = float(x)
# #                 pose.pose.position.y = float(y)
# #                 pose.pose.position.z = 0.0
# #                 pose.pose.orientation.w = 1.0
# #                 path_msg.poses.append(pose)

# #             self.publisher_.publish(path_msg)
# #             self.get_logger().info(f"BCD coverage path published with {len(continuous_pts)} points (cells: {len(cells)})")

# #             # stop timer if publish_once
# #             if self.publish_once:
# #                 self.timer.cancel()
# #                 self.get_logger().info("Planner: published once, timer stopped.")

# #         except Exception as e:
# #             self.get_logger().error(f"Error fetching or planning mission: {e}")

# #     # ------------------ helper geometry functions ------------------

# #     def _slice_polygon_into_strips(self, polygon_shape):
# #         """
# #         Slice polygon into horizontal strips between all unique vertex y-values.
# #         Return list of polygon pieces (shapely Polygons).
# #         """
# #         ys = sorted({round(y, 9) for (_, y) in polygon_shape.exterior.coords})  # unique vertex y's
# #         if len(ys) < 2:
# #             # degenerate polygon
# #             minx, miny, maxx, maxy = polygon_shape.bounds
# #             ys = [miny, maxy]

# #         minx, miny, maxx, maxy = polygon_shape.bounds
# #         pieces = []
# #         # Build strips between consecutive ys
# #         for i in range(len(ys) - 1):
# #             y0 = ys[i]
# #             y1 = ys[i+1]
# #             # Expand x a bit to avoid numerical boundary issues
# #             strip_box = box(minx - 1.0, y0, maxx + 1.0, y1)
# #             inter = polygon_shape.intersection(strip_box)
# #             if inter.is_empty:
# #                 continue
# #             # inter may be polygon or multipolygon
# #             if isinstance(inter, (Polygon,)):
# #                 pieces.append(inter)
# #             elif isinstance(inter, MultiPolygon):
# #                 for geom in inter.geoms:
# #                     pieces.append(geom)
# #         return pieces

# #     def _merge_pieces_into_cells(self, pieces):
# #         """
# #         Merge pieces into maximal connected components (cells).
# #         Two pieces are connected if they intersect or touch (with tiny buffer).
# #         Returns list of merged shapely Polygons (cells).
# #         """
# #         n = len(pieces)
# #         if n == 0:
# #             return []

# #         # Union-find / Disjoint set for connectivity
# #         parent = list(range(n))

# #         def find(a):
# #             while parent[a] != a:
# #                 parent[a] = parent[parent[a]]
# #                 a = parent[a]
# #             return a

# #         def union(a, b):
# #             ra = find(a); rb = find(b)
# #             if ra != rb:
# #                 parent[rb] = ra

# #         # connectivity test (use tiny buffer to be robust)
# #         for i in range(n):
# #             pi = pieces[i]
# #             for j in range(i+1, n):
# #                 pj = pieces[j]
# #                 # if they touch or overlap (buffer tiny to avoid floating edges)
# #                 if pi.buffer(1e-8).intersects(pj.buffer(1e-8)):
# #                     union(i, j)

# #         # Group by root
# #         groups = {}
# #         for idx in range(n):
# #             r = find(idx)
# #             groups.setdefault(r, []).append(idx)

# #         # Merge geometries inside each group
# #         cells = []
# #         for group_idxs in groups.values():
# #             geoms = [pieces[i] for i in group_idxs]
# #             merged = unary_union(geoms)
# #             # merged can be MultiPolygon if multiple disjoint parts ended up grouped; ensure list
# #             if isinstance(merged, Polygon):
# #                 cells.append(merged)
# #             elif isinstance(merged, MultiPolygon):
# #                 for g in merged.geoms:
# #                     cells.append(g)
# #         return cells

# #     def _generate_lawnmower_for_cell(self, cell_poly: Polygon, spacing=0.5, sample_step=0.25):
# #         """
# #         Given a shapely Polygon cell, generate ordered list of (x,y) points
# #         following a boustrophedon (lawnmower) pattern inside that cell.
# #         """
# #         minx, miny, maxx, maxy = cell_poly.bounds
# #         # We'll sweep along X (horizontal lines) with step = spacing in Y direction
# #         ys = []
# #         y = miny + 1e-6  # nudge inside
# #         while y <= maxy + 1e-6:
# #             ys.append(y)
# #             y += spacing

# #         segments = []
# #         direction = 1
# #         for y in ys:
# #             # create a horizontal line across bounds
# #             line = LineString([(minx - 1.0, y), (maxx + 1.0, y)])
# #             inter = cell_poly.intersection(line)
# #             # inter may be LineString or MultiLineString or empty
# #             if inter.is_empty:
# #                 direction *= -1
# #                 continue
# #             # convert to list of LineStrings
# #             if isinstance(inter, LineString):
# #                 lines = [inter]
# #             else:
# #                 # MultiLineString
# #                 lines = list(inter.geoms)

# #             # sort segments by x coordinate (left->right)
# #             lines_sorted = sorted(lines, key=lambda L: L.centroid.x)
# #             if direction < 0:
# #                 lines_sorted.reverse()

# #             # for each segment, sample points along it at sample_step
# #             for seg in lines_sorted:
# #                 pts = self._sample_linestring(seg, sample_step)
# #                 if pts:
# #                     segments.append(pts)
# #             direction *= -1

# #         # Flatten segments into single ordered path, alternating directions by construction above
# #         path_points = []
# #         for seg_pts in segments:
# #             # if previous point exists, ensure continuity by adding a transition point (stitching handled at higher level)
# #             path_points.extend(seg_pts)

# #         # Optional: simplify path (remove nearly-duplicate consecutive points)
# #         filtered = [path_points[0]] if path_points else []
# #         for p in path_points[1:]:
# #             if np.hypot(p[0] - filtered[-1][0], p[1] - filtered[-1][1]) > (sample_step * 0.5):
# #                 filtered.append(p)
# #         return filtered

# #     def _sample_linestring(self, ls: LineString, step):
# #         """
# #         Sample points along a LineString at approximately 'step' meters.
# #         Returns list of (x,y). Keeps endpoints.
# #         """
# #         length = ls.length
# #         if length == 0:
# #             return []
# #         n = max(1, int(np.ceil(length / step)))
# #         pts = []
# #         for i in range(n + 1):
# #             t = i / (n)
# #             pt = ls.interpolate(t, normalized=True)
# #             pts.append((pt.x, pt.y))
# #         return pts

# #     def _stitch_cell_paths(self, cell_paths):
# #         """
# #         Given a list of cell path lists (each an ordered list of (x,y) points),
# #         produce a single continuous path by ordering cells with a greedy nearest-neighbor
# #         and adding straight transition segments between endpoints.
# #         """
# #         if not cell_paths:
# #             return []

# #         # compute centroids for ordering
# #         centroids = [ (np.mean([p[0] for p in cp]), np.mean([p[1] for p in cp])) if cp else (0,0) for cp in cell_paths ]
# #         remaining = set(range(len(cell_paths)))
# #         # start from the first cell (arbitrary) - choose the cell containing polygon's first vertex would be nicer
# #         order = []
# #         cur = 0
# #         order.append(cur)
# #         remaining.remove(cur)
# #         while remaining:
# #             # find nearest remaining by centroid distance
# #             best = None; bestd = float('inf')
# #             cx, cy = centroids[cur]
# #             for r in remaining:
# #                 dx = centroids[r][0] - cx; dy = centroids[r][1] - cy
# #                 d = dx*dx + dy*dy
# #                 if d < bestd:
# #                     bestd = d; best = r
# #             order.append(best)
# #             remaining.remove(best)
# #             cur = best

# #         # build continuous path
# #         continuous = []
# #         for idx in order:
# #             cp = cell_paths[idx]
# #             if not cp:
# #                 continue
# #             if not continuous:
# #                 continuous.extend(cp)
# #             else:
# #                 # connect last point of continuous to first point of cp with straight-line interpolation
# #                 last = continuous[-1]; first = cp[0]
# #                 transition = self._linear_transition(last, first, step=self.sample_spacing)
# #                 # avoid duplicating start point if identical
# #                 if np.hypot(transition[0][0] - last[0], transition[0][1] - last[1]) > 1e-6:
# #                     continuous.extend(transition)
# #                 # then append cp
# #                 continuous.extend(cp)
# #         return continuous

# #     def _linear_transition(self, a, b, step=0.25):
# #         """
# #         Straight-line interpolation from a to b at 'step' spacing (excluding the starting point to avoid dup).
# #         Returns list of points starting from just after a and including b.
# #         """
# #         dx = b[0] - a[0]; dy = b[1] - a[1]
# #         dist = np.hypot(dx, dy)
# #         if dist < 1e-6:
# #             return [b]
# #         n = max(1, int(np.ceil(dist / step)))
# #         pts = []
# #         for i in range(1, n+1):
# #             t = i / n
# #             pts.append((a[0] + dx*t, a[1] + dy*t))
# #         return pts

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = BCDPlanner()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == "__main__":
# #     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import json
# import numpy as np
# from shapely.geometry import Polygon, Point

# class BCDPlanner(Node):
#     def __init__(self):
#         super().__init__('bcd_planner')

#         # Publisher for coverage path
#         self.publisher_ = self.create_publisher(Path, '/cleaning_coverage_path', 10)

#         # Subscribe to mission_in topic (published by MqttBridge)
#         self.subscription = self.create_subscription(
#             String,
#             'mission_in',
#             self.mission_callback,
#             10
#         )

#         # Store the latest polygon from mission
#         self.latest_polygon = None

#         # Timer to generate and publish coverage path every 2 seconds
#         self.timer = self.create_timer(2.0, self.generate_and_publish_path)

#         self.get_logger().info("BCD Planner node started and waiting for mission...")

#     def mission_callback(self, msg):
#         """
#         Receives mission from MQTT bridge and stores polygon coordinates.
#         Expected JSON format:
#         {
#             "geometry": {
#                 "coordinates": [
#                     [ [lon, lat], [lon, lat], ... ]
#                 ]
#             }
#         }
#         """
#         try:
#             mission = json.loads(msg.data)
#             coords = mission["geometry"]["coordinates"][0]  # first polygon
#             self.latest_polygon = [{"lat": c[1], "lon": c[0]} for c in coords]
#             self.get_logger().info(f"Received polygon with {len(self.latest_polygon)} points")
#         except Exception as e:
#             self.get_logger().error(f"Error parsing mission: {e}")

#     def generate_and_publish_path(self):
#         if not self.latest_polygon:
#             self.get_logger().warn("No polygon mission received yet...")
#             return

#         poly = self.latest_polygon
#         base_lat = poly[0]["lat"]
#         base_lon = poly[0]["lon"]
#         polygon_points_meters = []

#         # Convert lat/lon to local meters (relative to first point)
#         for pt in poly:
#             x = (pt["lon"] - base_lon) * 111000
#             y = (pt["lat"] - base_lat) * 111000
#             polygon_points_meters.append((x, y))

#         # Generate Boustrophedon coverage path (logic unchanged)
#         coverage_path = self.generate_coverage_path(polygon_points_meters, spacing=0.5)

#         # Convert coverage path to ROS Path message
#         path_msg = Path()
#         path_msg.header.frame_id = "odom"
#         path_msg.header.stamp = self.get_clock().now().to_msg()

#         for x, y in coverage_path:
#             pose = PoseStamped()
#             pose.header = path_msg.header
#             pose.pose.position.x = x
#             pose.pose.position.y = y
#             pose.pose.position.z = 0.0
#             pose.pose.orientation.w = 1.0
#             path_msg.poses.append(pose)

#         self.publisher_.publish(path_msg)
#         self.get_logger().info(f"Coverage path published with {len(coverage_path)} points")

#     def generate_coverage_path(self, points_list, spacing=0.5):
#         """
#         Generates a simple lawnmower/Boustrophedon coverage path
#         inside the polygon using Shapely for robust point-in-polygon tests.
#         """
#         polygon_shape = Polygon(points_list)
#         min_x, min_y, max_x, max_y = polygon_shape.bounds

#         coverage_path = []
#         y = min_y
#         direction = 1  # 1 = left to right, -1 = right to left

#         while y <= max_y:
#             if direction == 1:
#                 xs = np.arange(min_x, max_x, spacing)
#             else:
#                 xs = np.arange(max_x, min_x, -spacing)

#             for x in xs:
#                 if polygon_shape.contains(Point(x, y)):
#                     coverage_path.append((x, y))

#             y += spacing
#             direction *= -1

#         return coverage_path


# def main(args=None):
#     rclpy.init(args=args)
#     node = BCDPlanner()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


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
    """
    Convert a yaw (radians) into a quaternion message (only yaw used).
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class BCDPlanner(Node):
    """
    Boustrophedon Coverage Decomposition (simple lawnmower) planner.

    Important notes:
    - The path is generated in a **local metric frame** where (0,0) corresponds
      to the first polygon vertex (base_lat, base_lon). The header.frame_id
      is set to `local_frame` (default: "local_map"). Ensure whatever follows
      this Path (controllers or transforms) expects coordinates in that frame,
      or transform accordingly.
    - Call generate_coverage_path only once when a new mission arrives.
    """

    def __init__(self):
        super().__init__('bcd_planner')

        # Parameters (tweak these as needed)
        self.declare_parameter("spacing_m", 0.5)        # meters between adjacent sweep lines
        self.declare_parameter("max_points", 2000)      # cap for published waypoints
        self.declare_parameter("local_frame", "odom")  # publish path in 'odom' frame for RViz
 # frame for path message
        self.declare_parameter("min_spacing", 0.1)      # minimum allowed spacing

        self.spacing_m = float(self.get_parameter("spacing_m").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.local_frame = str(self.get_parameter("local_frame").value)
        self.min_spacing = float(self.get_parameter("min_spacing").value)

        # Publisher for the coverage path (published once per mission)
        self.publisher_ = self.create_publisher(Path, '/cleaning_coverage_path', 10)

        # Subscribe to mission_in topic (published by MqttBridge)
        self.subscription = self.create_subscription(
            String,
            'mission_in',
            self.mission_callback,
            10
        )

        # store polygon as list of dict {lat, lon}
        self.latest_polygon = None

        self.get_logger().info("BCD Planner node started and waiting for mission...")

    def mission_callback(self, msg: String):
        """
        Called when mission JSON arrives (from MQTT bridge).
        Expected JSON format:
        {
            "geometry": {
                "coordinates": [
                    [ [lon, lat], [lon, lat], ... ]   # polygon ring (lon, lat)
                ]
            }
        }

        On receipt the planner:
        - converts to local meters (ENU-like, relative to first vertex)
        - generates coverage path once
        - publishes path as a nav_msgs/Path
        """
        try:
            mission = json.loads(msg.data)
            coords = mission["geometry"]["coordinates"][0]  # first polygon ring
            if len(coords) < 3:
                self.get_logger().error("Received polygon has fewer than 3 points.")
                return

            # store as dict list
            self.latest_polygon = [{"lat": float(c[1]), "lon": float(c[0])} for c in coords]
            self.get_logger().info(f"Received polygon with {len(self.latest_polygon)} points")

            # Generate and publish path now (only once per mission)
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

        # meter-per-degree approximations
        meters_per_deg_lat = 111_132  # approximate
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

        # Downsample if too many points to keep within max_points
        if len(raw_path) > self.max_points:
            # uniform downsample to avoid biasing start/end
            indices = np.linspace(0, len(raw_path) - 1, self.max_points).astype(int)
            sampled_path = [raw_path[i] for i in indices]
            self.get_logger().info(f"Downsampled path from {len(raw_path)} to {len(sampled_path)} points (max_points={self.max_points}).")
        else:
            sampled_path = raw_path

        # Build ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = self.local_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Create PoseStamped for each waypoint with orientation towards next waypoint
        for i, (x, y) in enumerate(sampled_path):
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # compute yaw towards next point when possible, otherwise keep last yaw
            if i < len(sampled_path) - 1:
                nx, ny = sampled_path[i + 1]
                dx = nx - x
                dy = ny - y
                yaw = math.atan2(dy, dx)
            else:
                # for last point, reuse previous yaw or 0
                if i > 0:
                    px, py = sampled_path[i - 1]
                    yaw = math.atan2(y - py, x - px)
                else:
                    yaw = 0.0

            pose.pose.orientation = yaw_to_quaternion(yaw)
            path_msg.poses.append(pose)

        # Publish once
        self.publisher_.publish(path_msg)
        self.get_logger().info(f"Published coverage path with {len(path_msg.poses)} waypoints in frame '{self.local_frame}'.")

    def generate_coverage_path(self, prepared_poly: 'PreparedGeometry', bounds: Tuple[float, float, float, float], spacing: float) -> List[Tuple[float, float]]:
        """
        Generate a boustrophedon (lawnmower) path inside the polygon.

        - Uses prepared_poly.covers(Point(x,y)) for boundary-including tests.
        - Sweeps along the longer dimension for efficiency.
        """
        min_x, min_y, max_x, max_y = bounds
        width = max_x - min_x
        height = max_y - min_y

        # choose sweep direction: sweep along x if width < height (i.e., lines run east-west),
        # else sweep along y (lines run north-south). This reduces number of line segments.
        sweep_along_x = width < height

        path: List[Tuple[float, float]] = []
        direction = 1  # alternate direction each sweep

        # Compute number of lines and ensure we include max bound
        if sweep_along_x:
            # lines vary in y
            num_lines = max(1, int(math.ceil(height / spacing)))
            ys = np.linspace(min_y, max_y, num_lines + 1)
            for row_index, y in enumerate(ys):
                if direction == 1:
                    xs = np.arange(min_x, max_x + 1e-6, spacing)
                else:
                    xs = np.arange(max_x, min_x - 1e-6, -spacing)

                for x in xs:
                    pt = Point(float(x), float(y))
                    if prepared_poly.covers(pt):
                        path.append((float(x), float(y)))
                direction *= -1
        else:
            # lines vary in x
            num_lines = max(1, int(math.ceil(width / spacing)))
            xs = np.linspace(min_x, max_x, num_lines + 1)
            for col_index, x in enumerate(xs):
                if direction == 1:
                    ys = np.arange(min_y, max_y + 1e-6, spacing)
                else:
                    ys = np.arange(max_y, min_y - 1e-6, -spacing)

                for y in ys:
                    pt = Point(float(x), float(y))
                    if prepared_poly.covers(pt):
                        path.append((float(x), float(y)))
                direction *= -1

        # remove consecutive duplicate points (if any)
        if path:
            compacted = [path[0]]
            for p in path[1:]:
                if abs(p[0] - compacted[-1][0]) > 1e-6 or abs(p[1] - compacted[-1][1]) > 1e-6:
                    compacted.append(p)
            path = compacted

        self.get_logger().info(f"Generated raw coverage path with {len(path)} candidate points (spacing={spacing}m).")
        return path


def main(args=None):
    rclpy.init(args=args)
    node = BCDPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down BCD Planner (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
