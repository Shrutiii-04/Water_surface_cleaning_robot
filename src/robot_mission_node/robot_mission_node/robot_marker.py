#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SweepingMarker(Node):
    def __init__(self):
        super().__init__('sweeping_marker_fixed')

        self.declare_parameter("speed_m_s", 0.5) 
        self.declare_parameter("timer_hz", 20.0)         
        self.declare_parameter("loop_path", True)
        self.declare_parameter("marker_scale", 0.2) 
        self.declare_parameter("use_path_orientation", False)
        self.declare_parameter("marker_ns", "sweep_robot")
        self.declare_parameter("marker_id", 0)

        self.speed_m_s: float = float(self.get_parameter("speed_m_s").value)
        timer_hz: float = float(self.get_parameter("timer_hz").value)
        self.loop_path: bool = bool(self.get_parameter("loop_path").value)
        self.marker_scale: float = float(self.get_parameter("marker_scale").value)
        self.use_path_orientation: bool = bool(self.get_parameter("use_path_orientation").value)
        self.marker_ns: str = str(self.get_parameter("marker_ns").value)
        self.marker_id: int = int(self.get_parameter("marker_id").value)

        period = 1.0 / max(1.0, timer_hz)

        # Subscriber & publisher
        self.path_sub = self.create_subscription(Path, '/cleaning_coverage_path', self.path_cb, 10)
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)

        # internal state for interpolation along path
        self._points: List[Tuple[float, float]] = []  
        self._poses_orientations: List[Optional[Quaternion]] = []
        self._frame_id: str = "odom"
        self._seg_idx: int = 0
        self._seg_progress: float = 0.0
        self._seg_length: float = 0.0
        self._last_update_time = self.get_clock().now()
        self.get_logger().info("SweepingMarker (fixed) started")

        self.timer = self.create_timer(period, self._timer_cb)

    def path_cb(self, msg: Path):
        """
        Called when a Path is received. Stores points and optional per-pose orientations.
        Resets interpolation so marker starts from path start.
        """
        # store frame to use for marker header
        if msg.header.frame_id:
            self._frame_id = msg.header.frame_id

        # extract points and optional orientations
        new_points: List[Tuple[float, float]] = []
        new_orients: List[Optional[Quaternion]] = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            new_points.append((x, y))
            # store orientation only if present and non-default
            orient = pose_stamped.pose.orientation
            if (orient.w != 0.0 or orient.x != 0.0 or orient.y != 0.0 or orient.z != 0.0):
                new_orients.append(orient)
            else:
                new_orients.append(None)

        # If path didn't change, do nothing
        if new_points == self._points:
            return

        # Use new path - reset interpolation state
        self._points = new_points
        self._poses_orientations = new_orients
        self._seg_idx = 0
        self._seg_progress = 0.0
        self._seg_length = 0.0
        self._last_update_time = self.get_clock().now()=
        if len(self._points) >= 2:
            self._seg_length = self._compute_seg_length(self._seg_idx)
        else:
            self._seg_length = 0.0

        self.get_logger().info(f"Received coverage path with {len(self._points)} points (frame='{self._frame_id}')")

    def _compute_seg_length(self, idx: int) -> float:
        if idx < 0 or idx >= len(self._points) - 1:
            return 0.0
        x0, y0 = self._points[idx]
        x1, y1 = self._points[idx + 1]
        return math.hypot(x1 - x0, y1 - y0)

    def _advance_along_path(self, dt: float):
        """
        Move the internal interpolation state forward by (speed * dt) meters.
        This advances across multiple segments if necessary.
        """
        if not self._points or len(self._points) == 1:
            return

        remaining = self.speed_m_s * max(0.0, dt)

        # if segment length zero (coincident points), skip to next
        while remaining > 0 and len(self._points) >= 2:
            if self._seg_length <= 1e-9:
                # move to next segment
                if self._seg_idx < len(self._points) - 2:
                    self._seg_idx += 1
                    self._seg_length = self._compute_seg_length(self._seg_idx)
                    self._seg_progress = 0.0
                    continue
                else:
                    # at last point already
                    self._seg_progress = 1.0
                    remaining = 0.0
                    break

            # distance left on current segment
            dist_on_seg = (1.0 - self._seg_progress) * self._seg_length
            if remaining < dist_on_seg - 1e-9:
                # advance within current segment
                self._seg_progress += remaining / self._seg_length
                remaining = 0.0
            else:
                # consume the rest of this segment and move to next
                remaining -= dist_on_seg
                if self._seg_idx < len(self._points) - 2:
                    self._seg_idx += 1
                    self._seg_length = self._compute_seg_length(self._seg_idx)
                    self._seg_progress = 0.0
                else:
                    # reached end of path
                    self._seg_progress = 1.0
                    if self.loop_path:
                        # loop back to start
                        self._seg_idx = 0
                        self._seg_progress = 0.0
                        self._seg_length = self._compute_seg_length(self._seg_idx) if len(self._points) >= 2 else 0.0
                        # continue if remaining still > 0
                    else:
                        remaining = 0.0
                        break

    def _current_position_and_yaw(self) -> Tuple[Tuple[float, float], float]:
        """
        Returns current interpolated (x,y) and yaw (radians).
        Yaw is computed from motion direction. If use_path_orientation is True and the
        current pose in the path contains an orientation, that orientation is preferred.
        """
        if not self._points:
            return (0.0, 0.0), 0.0
        if len(self._points) == 1:
            x, y = self._points[0]
            yaw = 0.0
            return (x, y), yaw

        i = self._seg_idx
        p0 = self._points[i]
        p1 = self._points[i + 1]
        t = max(0.0, min(1.0, self._seg_progress))
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t

        if self.use_path_orientation:
            orient = self._poses_orientations[i] if i < len(self._poses_orientations) else None
            if orient:
                # convert quaternion to yaw
                yaw = math.atan2(2.0 * (orient.w * orient.z + orient.x * orient.y),
                                 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z))
                return (x, y), yaw

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            yaw = 0.0
        else:
            yaw = math.atan2(dy, dx)
        return (x, y), yaw

    def _timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds * 1e-9
        self._last_update_time = now

        # if no path, publish nothing
        if not self._points:
            return

        # advance interpolation by dt
        self._advance_along_path(dt)

        # get current position and yaw
        (x, y), yaw = self._current_position_and_yaw()

        # create marker
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = now.to_msg()
        marker.ns = self.marker_ns
        marker.id = self.marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.1

        # set orientation
        marker.pose.orientation = yaw_to_quaternion(yaw)

        # scale & color
        s = float(self.marker_scale)
        marker.scale.x = s
        marker.scale.y = s
        marker.scale.z = s

        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 1.0

        self.marker_pub.publish(marker)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SweepingMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SweepingMarker (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
