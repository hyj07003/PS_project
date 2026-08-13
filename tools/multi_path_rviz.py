#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from traffic_common import find_path_conflicts, path_points


class MultiPathRvizBridge(Node):
    """
    Poll each Pinky Flask /nav/path and publish RViz-friendly multi-robot traffic topics.

    Published topics:
      /robot1/plan                nav_msgs/Path
      /robot2/plan                nav_msgs/Path
      /multi_robot/traffic_markers visualization_msgs/MarkerArray

    traffic_markers contains:
      - Robot1 global path (green line)
      - Robot2 global path (blue line)
      - path-overlap/conflict samples (red spheres)
      - CONFLICT / CLEAR text marker
    """

    def __init__(
        self,
        robot1_url: str,
        robot2_url: str,
        hz: float = 5.0,
        clearance_m: float = 0.30,
    ):
        super().__init__("multi_path_rviz_bridge")
        self._robots = {
            "robot1": robot1_url.rstrip("/"),
            "robot2": robot2_url.rstrip("/"),
        }
        self._pubs = {
            "robot1": self.create_publisher(Path, "/robot1/plan", 10),
            "robot2": self.create_publisher(Path, "/robot2/plan", 10),
        }
        self._marker_pub = self.create_publisher(
            MarkerArray,
            "/multi_robot/traffic_markers",
            10,
        )
        self._clearance_m = max(0.01, float(clearance_m))
        self._last_count = {"robot1": -1, "robot2": -1}
        self._last_conflict: bool | None = None
        self.create_timer(1.0 / max(0.2, hz), self._tick)
        self.get_logger().info(
            "RViz traffic bridge: /robot1/plan, /robot2/plan, "
            "/multi_robot/traffic_markers"
        )
        self.get_logger().info(
            f"Conflict clearance = {self._clearance_m:.2f} m"
        )

    def _get_path(self, base_url: str) -> dict | None:
        req = urllib.request.Request(f"{base_url}/nav/path", method="GET")
        try:
            with urllib.request.urlopen(req, timeout=0.8) as res:
                payload = json.loads(res.read().decode("utf-8"))
        except (urllib.error.URLError, TimeoutError, OSError, ValueError):
            return None
        path = payload.get("path") if isinstance(payload, dict) else None
        return path if isinstance(path, dict) else None

    def _to_ros_path(self, payload: dict) -> Path:
        msg = Path()
        msg.header.frame_id = str(payload.get("frameId") or "map")
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in payload.get("poses") or []:
            if not isinstance(p, dict):
                continue
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(p.get("x") or 0.0)
            pose.pose.position.y = float(p.get("y") or 0.0)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def _line_marker(
        self,
        marker_id: int,
        namespace: str,
        frame_id: str,
        points_xy: list[tuple[float, float]],
        rgb: tuple[float, float, float],
        z: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 0.95
        for x, y in points_xy:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = z
            marker.points.append(p)
        return marker

    def _conflict_marker(
        self,
        frame_id: str,
        samples: list[dict],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "conflict_zone"
        marker.id = 10
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        # Diameter of each highlighted conflict sample.
        marker.scale.x = self._clearance_m
        marker.scale.y = self._clearance_m
        marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.08
        marker.color.b = 0.05
        marker.color.a = 0.72
        for sample in samples:
            p = Point()
            p.x = float(sample.get("x", 0.0))
            p.y = float(sample.get("y", 0.0))
            p.z = 0.10
            marker.points.append(p)
        return marker

    def _status_marker(
        self,
        frame_id: str,
        conflict: bool,
        samples: list[dict],
        path1: list[tuple[float, float]],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "traffic_status"
        marker.id = 20
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.20
        marker.color.a = 1.0

        if conflict and samples:
            marker.text = "CONFLICT"
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.05
            marker.pose.position.x = float(samples[0].get("x", 0.0))
            marker.pose.position.y = float(samples[0].get("y", 0.0))
            marker.pose.position.z = 0.35
        else:
            marker.text = "PATHS CLEAR"
            marker.color.r = 0.10
            marker.color.g = 0.90
            marker.color.b = 0.20
            if path1:
                marker.pose.position.x = path1[0][0]
                marker.pose.position.y = path1[0][1]
            marker.pose.position.z = 0.35
        return marker

    def _delete_all_marker(self, frame_id: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return marker

    def _publish_visualization(self, payload1: dict, payload2: dict) -> None:
        p1 = path_points(payload1)
        p2 = path_points(payload2)
        if not p1 and not p2:
            return

        frame1 = str(payload1.get("frameId") or "map")
        frame2 = str(payload2.get("frameId") or "map")
        if frame1 != frame2:
            self.get_logger().warning(
                f"Cannot compare paths in different frames: {frame1} vs {frame2}"
            )
            return

        conflict = find_path_conflicts(
            p1,
            p2,
            clearance_m=self._clearance_m,
            max_samples=200,
        )
        samples = conflict.get("samples") or []

        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker(frame1))
        if p1:
            markers.markers.append(
                self._line_marker(1, "robot1_path", frame1, p1, (0.10, 0.95, 0.20), 0.05)
            )
        if p2:
            markers.markers.append(
                self._line_marker(2, "robot2_path", frame1, p2, (0.10, 0.45, 1.00), 0.07)
            )
        if samples:
            markers.markers.append(self._conflict_marker(frame1, samples))
        markers.markers.append(
            self._status_marker(frame1, bool(conflict.get("conflict")), samples, p1)
        )
        self._marker_pub.publish(markers)

        current_conflict = bool(conflict.get("conflict"))
        if self._last_conflict is None or current_conflict != self._last_conflict:
            min_dist = conflict.get("minDistanceM")
            min_text = "n/a" if min_dist is None else f"{min_dist:.3f} m"
            if current_conflict:
                self.get_logger().warning(
                    f"PATH CONFLICT: {len(samples)} samples, min distance={min_text}"
                )
            else:
                self.get_logger().info(
                    f"PATHS CLEAR: min distance={min_text}"
                )
            self._last_conflict = current_conflict

    def _tick(self) -> None:
        payloads: dict[str, dict] = {}
        for name, base_url in self._robots.items():
            payload = self._get_path(base_url)
            if payload is None:
                continue
            payloads[name] = payload
            msg = self._to_ros_path(payload)
            if not msg.poses:
                continue
            self._pubs[name].publish(msg)
            count = len(msg.poses)
            if self._last_count[name] != count:
                self.get_logger().info(f"{name}: publish {count} points")
                self._last_count[name] = count

        if "robot1" in payloads and "robot2" in payloads:
            self._publish_visualization(payloads["robot1"], payloads["robot2"])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot1", required=True, help="http://ROBOT1_IP:4200")
    parser.add_argument("--robot2", required=True, help="http://ROBOT2_IP:4200")
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="path polling / RViz refresh rate [Hz]",
    )
    parser.add_argument(
        "--clearance",
        type=float,
        default=0.30,
        help="distance below which two paths are highlighted as conflicting [m]",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MultiPathRvizBridge(
        args.robot1,
        args.robot2,
        hz=args.hz,
        clearance_m=args.clearance,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
