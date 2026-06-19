#!/usr/bin/env python3

from __future__ import annotations

import math
from time import perf_counter
from typing import List

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from line_fitting.split_and_merge import (
    LineSegment,
    extract_lines,
    point_chunks_from_scan,
)


class LineFittingNode(Node):
    def __init__(self) -> None:
        super().__init__("line_fitting_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_topic", "/line_markers")
        self.declare_parameter("split_threshold", 0.04)
        self.declare_parameter("merge_threshold", 0.04)
        self.declare_parameter("min_points", 6)
        self.declare_parameter("max_point_gap", 0.25)
        self.declare_parameter("log_period", 1.0)

        scan_topic = str(self.get_parameter("scan_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)

        self.marker_publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.callback,
            qos_profile_sensor_data,
        )

        self.last_log_time = 0.0
        self.get_logger().info(
            "Ready for iterative Split-and-Merge line extraction "
            f"(scan_topic={scan_topic}, marker_topic={marker_topic})."
        )

    def callback(self, data: LaserScan) -> None:
        split_threshold = float(self.get_parameter("split_threshold").value)
        merge_threshold = float(self.get_parameter("merge_threshold").value)
        min_points = max(2, int(self.get_parameter("min_points").value))
        max_point_gap = float(self.get_parameter("max_point_gap").value)

        point_chunks = point_chunks_from_scan(
            ranges=data.ranges,
            angle_min=data.angle_min,
            angle_increment=data.angle_increment,
            range_min=data.range_min,
            range_max=data.range_max,
            max_point_gap=max_point_gap,
            min_points=min_points,
        )

        start_time = perf_counter()
        lines = extract_lines(
            point_chunks,
            split_threshold=split_threshold,
            merge_threshold=merge_threshold,
            min_points=min_points,
        )
        elapsed_ms = (perf_counter() - start_time) * 1000.0

        self.publish_markers(data, lines)
        self.log_result_summary(lines, elapsed_ms)

    def publish_markers(self, scan: LaserScan, lines: List[LineSegment]) -> None:
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.stamp = scan.header.stamp
        delete_marker.header.frame_id = scan.header.frame_id or "base_scan"
        delete_marker.ns = "split_and_merge_lines"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for index, line in enumerate(lines, start=1):
            marker = Marker()
            marker.header.stamp = scan.header.stamp
            marker.header.frame_id = scan.header.frame_id or "base_scan"
            marker.ns = "split_and_merge_lines"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03
            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 1.0
            marker.points = [
                Point(x=line.start_point[0], y=line.start_point[1], z=0.02),
                Point(x=line.end_point[0], y=line.end_point[1], z=0.02),
            ]
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def log_result_summary(
        self,
        lines: List[LineSegment],
        elapsed_ms: float,
    ) -> None:
        log_period = max(0.0, float(self.get_parameter("log_period").value))
        current_time = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if log_period > 0.0 and current_time - self.last_log_time < log_period:
            return

        self.last_log_time = current_time

        line_parts = []
        for index, line in enumerate(lines, start=1):
            line_parts.append(
                f"{index:02d}: rho={line.rho:.3f} m, "
                f"alpha={line.alpha:.3f} rad ({math.degrees(line.alpha):.1f} deg), "
                f"points={line.point_count}, max_error={line.max_error:.3f} m"
            )

        if not line_parts:
            line_parts.append("No lines detected.")

        self.get_logger().info(
            f"Iterative Split-and-Merge: {len(lines)} lines, {elapsed_ms:.3f} ms"
            + "\nLine parameters:\n"
            + "\n".join(line_parts)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFittingNode()
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
