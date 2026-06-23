#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import numpy
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Int32
from visualization_msgs.msg import Marker, MarkerArray

from ekf_line_localization.ekf import (
    associate_measurements, # inside this function, the following functions are called: measurement_function, measurement_innovation, mahalanobis_distance
    filter_step,
    load_line_map,
    load_line_segments,
    predict_covariance,
    transition_function,
)
from line_fitting.split_and_merge import extract_lines, point_chunks_from_scan


LEFT_WHEEL_JOINT = "wheel_left_joint"
RIGHT_WHEEL_JOINT = "wheel_right_joint"


@dataclass
class PredictionRecord:
    stamp: float
    stamp_msg: object
    left_delta: float
    right_delta: float
    state_after: numpy.ndarray
    covariance_after: numpy.ndarray


def default_share_path(*parts: str) -> str:
    try:
        base_path = Path(get_package_share_directory("ekf_line_localization"))
    except PackageNotFoundError:
        base_path = Path(__file__).resolve().parents[1]
    return str(base_path.joinpath(*parts))


def stamp_to_seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class EkfLineLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_line_localization")

        default_map_file = default_share_path("config", "turtlebot3_maze_lines.yaml")

        self.declare_parameter("map_file", default_map_file)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("initial_covariance", [0.02, 0.02, 0.05])
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.160)
        self.declare_parameter("motion_noise_gain", 0.02)
        self.declare_parameter("sigma_alpha", 0.05)
        self.declare_parameter("sigma_r", 0.02)
        self.declare_parameter("validation_gate", 5.0)
        self.declare_parameter("enable_correction", True)
        self.declare_parameter("history_size", 500)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("pose_topic", "/ekf_pose")
        self.declare_parameter("association_count_topic", "/ekf_association_count")
        self.declare_parameter("update_applied_topic", "/ekf_update_applied")
        self.declare_parameter("map_marker_topic", "/ekf_map_lines")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_map_markers", True)
        self.declare_parameter("map_marker_length", 7.0)
        self.declare_parameter("split_threshold", 0.04)
        self.declare_parameter("merge_threshold", 0.04)
        self.declare_parameter("min_points", 6)
        self.declare_parameter("max_point_gap", 0.25)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.motion_noise_gain = float(self.get_parameter("motion_noise_gain").value)
        self.validation_gate = float(self.get_parameter("validation_gate").value)
        self.history_size = max(1, int(self.get_parameter("history_size").value))

        self.state = numpy.asarray(
            self.get_parameter("initial_pose").value,
            dtype=float,
        ).reshape(3)
        self.covariance = numpy.diag(
            numpy.asarray(
                self.get_parameter("initial_covariance").value,
                dtype=float,
            ).reshape(3)
        )

        sigma_alpha = float(self.get_parameter("sigma_alpha").value)
        sigma_r = float(self.get_parameter("sigma_r").value)
        self.measurement_covariance = numpy.diag([sigma_alpha**2, sigma_r**2])

        map_file = str(self.get_parameter("map_file").value)
        self.map_lines = load_line_map(map_file)
        self.map_segments = load_line_segments(map_file)
        if len(self.map_segments) not in (0, len(self.map_lines)):
            self.get_logger().warning(
                "Map visualization segments count does not match line count; "
                "falling back to generated line markers."
            )
            self.map_segments = numpy.zeros((0, 2, 2), dtype=float)

        pose_topic = str(self.get_parameter("pose_topic").value)
        association_topic = str(self.get_parameter("association_count_topic").value)
        update_topic = str(self.get_parameter("update_applied_topic").value)
        map_marker_topic = str(self.get_parameter("map_marker_topic").value)

        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            pose_topic,
            10,
        )
        self.association_count_publisher = self.create_publisher( # This tells how many map lines were successfully associated with observed laser lines in that scan cycle.
            Int32,
            association_topic,
            10,
        )
        self.update_applied_publisher = self.create_publisher(Bool, update_topic, 10) # This tells whether the EKF correction step actually happened.
        self.map_marker_publisher = self.create_publisher( # This tells whether the EKF correction step actually happened.
            MarkerArray,
            map_marker_topic,
            10,
        )

        self.previous_left_position: Optional[float] = None
        self.previous_right_position: Optional[float] = None
        self.prediction_records: list[PredictionRecord] = []
        self.last_joint_warning_time = 0.0
        self.map_markers_initialized = False

        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)

        self.joint_subscription = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        if bool(self.get_parameter("publish_map_markers").value):
            self.map_marker_timer = self.create_timer(2.0, self.publish_map_markers)

        self.publish_pose(self.get_clock().now().to_msg())
        self.publish_map_markers()
        self.get_logger().info(
            "Line EKF localization ready "
            f"(map_lines={len(self.map_lines)}, pose_topic={pose_topic}, "
            f"correction={bool(self.get_parameter('enable_correction').value)})."
        )

    def joint_state_callback(self, message: JointState) -> None:
        wheel_positions = self.extract_wheel_positions(message)
        if wheel_positions is None:
            return

        left_position, right_position = wheel_positions
        if self.previous_left_position is None or self.previous_right_position is None:
            self.previous_left_position = left_position
            self.previous_right_position = right_position
            self.publish_pose(message.header.stamp)
            return

        left_delta = self.wheel_radius * (left_position - self.previous_left_position)
        right_delta = self.wheel_radius * (right_position - self.previous_right_position)
        self.previous_left_position = left_position
        self.previous_right_position = right_position

        self.apply_prediction(left_delta, right_delta)
        record = PredictionRecord(
            stamp=stamp_to_seconds(message.header.stamp),
            stamp_msg=message.header.stamp,
            left_delta=left_delta,
            right_delta=right_delta,
            state_after=self.state.copy(),
            covariance_after=self.covariance.copy(),
        )
        self.prediction_records.append(record)
        del self.prediction_records[: max(0, len(self.prediction_records) - self.history_size)]

        self.publish_pose(message.header.stamp)

    def scan_callback(self, message: LaserScan) -> None:
        if not bool(self.get_parameter("enable_correction").value):
            self.publish_update_result(0, False)
            return

        if not self.prediction_records:
            self.publish_update_result(0, False)
            return

        observations = self.extract_line_observations(message)
        if observations.size == 0:
            self.publish_update_result(0, False)
            return

        record_index = self.find_record_index(stamp_to_seconds(message.header.stamp))
        prior_record = self.prediction_records[record_index] # joint

        association = associate_measurements(
            prior_record.state_after,
            prior_record.covariance_after,
            observations,
            self.measurement_covariance,
            self.map_lines,
            self.validation_gate,
        )

        if association.count == 0:
            self.publish_update_result(0, False)  # count = 0, update_applied = False means no valid line association, so no correction.
            return

        corrected_state, corrected_covariance = filter_step(
            prior_record.state_after,
            prior_record.covariance_after,
            association.innovation,
            association.jacobian,
            association.covariance,
        )
        self.repropagate_from_record(
            record_index,
            corrected_state,
            corrected_covariance,
        )

        self.publish_update_result(association.count, True)
        self.publish_pose(self.prediction_records[-1].stamp_msg)

    def extract_wheel_positions(
        self,
        message: JointState,
    ) -> Optional[tuple[float, float]]:
        try:
            left_index = message.name.index(LEFT_WHEEL_JOINT)
            right_index = message.name.index(RIGHT_WHEEL_JOINT)
        except ValueError:
            self.warn_missing_wheels()
            return None

        if left_index >= len(message.position) or right_index >= len(message.position):
            self.warn_missing_wheels()
            return None

        return float(message.position[left_index]), float(message.position[right_index])

    def warn_missing_wheels(self) -> None:
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if current_time - self.last_joint_warning_time < 2.0:
            return
        self.last_joint_warning_time = current_time
        self.get_logger().warning(
            "JointState does not contain wheel_left_joint and wheel_right_joint."
        )

    def extract_line_observations(self, message: LaserScan) -> numpy.ndarray:
        split_threshold = float(self.get_parameter("split_threshold").value)
        merge_threshold = float(self.get_parameter("merge_threshold").value)
        min_points = max(2, int(self.get_parameter("min_points").value))
        max_point_gap = float(self.get_parameter("max_point_gap").value)

        chunks = point_chunks_from_scan(
            ranges=message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            max_point_gap=max_point_gap,
            min_points=min_points,
        )
        lines = extract_lines(
            chunks,
            split_threshold=split_threshold,
            merge_threshold=merge_threshold,
            min_points=min_points,
        )
        if not lines:
            return numpy.zeros((0, 2), dtype=float)
        return numpy.asarray([[line.alpha, line.rho] for line in lines], dtype=float)

    def apply_prediction(self, left_delta: float, right_delta: float) -> None:
        control = numpy.array([left_delta, right_delta], dtype=float)
        predicted_state, fx, fu = transition_function(
            self.state,
            control,
            self.wheel_separation,
        )
        predicted_covariance = predict_covariance(
            self.covariance,
            fx,
            fu,
            control,
            self.motion_noise_gain,
        )
        self.state = predicted_state
        self.covariance = predicted_covariance

    def find_record_index(self, stamp: float) -> int:
        record_index = 0
        for index, record in enumerate(self.prediction_records):
            if record.stamp <= stamp:
                record_index = index
            else:
                break
        return record_index

    def repropagate_from_record(
        self,
        record_index: int,
        corrected_state: numpy.ndarray,
        corrected_covariance: numpy.ndarray,
    ) -> None:
        self.prediction_records[record_index].state_after = corrected_state.copy()
        self.prediction_records[record_index].covariance_after = corrected_covariance.copy()

        state = corrected_state.copy()
        covariance = corrected_covariance.copy()
        for index in range(record_index + 1, len(self.prediction_records)):
            record = self.prediction_records[index]
            control = numpy.array([record.left_delta, record.right_delta], dtype=float)
            state, fx, fu = transition_function(state, control, self.wheel_separation)
            covariance = predict_covariance(
                covariance,
                fx,
                fu,
                control,
                self.motion_noise_gain,
            )
            record.state_after = state.copy()
            record.covariance_after = covariance.copy()

        self.state = self.prediction_records[-1].state_after.copy()
        self.covariance = self.prediction_records[-1].covariance_after.copy()

    def publish_pose(self, stamp: object) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.pose.pose.position.x = float(self.state[0])
        message.pose.pose.position.y = float(self.state[1])
        message.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(self.state[2]))
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw

        covariance = [0.0] * 36
        covariance[0] = float(self.covariance[0, 0])
        covariance[1] = float(self.covariance[0, 1])
        covariance[5] = float(self.covariance[0, 2])
        covariance[6] = float(self.covariance[1, 0])
        covariance[7] = float(self.covariance[1, 1])
        covariance[11] = float(self.covariance[1, 2])
        covariance[30] = float(self.covariance[2, 0])
        covariance[31] = float(self.covariance[2, 1])
        covariance[35] = float(self.covariance[2, 2])
        message.pose.covariance = covariance

        self.pose_publisher.publish(message)

    def publish_update_result(self, count: int, update_applied: bool) -> None:
        count_message = Int32()
        count_message.data = int(count)
        self.association_count_publisher.publish(count_message)

        update_message = Bool()
        update_message.data = bool(update_applied)
        self.update_applied_publisher.publish(update_message)

    def publish_map_markers(self) -> None:
        marker_array = MarkerArray()
        marker_stamp = self.get_clock().now().to_msg()

        if not self.map_markers_initialized:
            delete_marker = Marker()
            delete_marker.header.stamp = marker_stamp
            delete_marker.header.frame_id = self.frame_id
            delete_marker.ns = "ekf_line_map"
            delete_marker.id = 0
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.map_markers_initialized = True

        marker_length = float(self.get_parameter("map_marker_length").value)
        use_map_segments = len(self.map_segments) == len(self.map_lines)
        for line_index, line in enumerate(self.map_lines):
            if use_map_segments:
                start, end = self.map_segments[line_index]
            else:
                alpha, radius = line
                normal = numpy.array([math.cos(alpha), math.sin(alpha)], dtype=float)
                direction = numpy.array([-math.sin(alpha), math.cos(alpha)], dtype=float)
                center = radius * normal
                start = center - 0.5 * marker_length * direction
                end = center + 0.5 * marker_length * direction

            marker = Marker()
            marker.header.stamp = marker_stamp
            marker.header.frame_id = self.frame_id
            marker.ns = "ekf_line_map"
            marker.id = line_index + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.color.r = 0.0
            marker.color.g = 0.35
            marker.color.b = 1.0
            marker.color.a = 0.65
            marker.points = [
                Point(x=float(start[0]), y=float(start[1]), z=0.04),
                Point(x=float(end[0]), y=float(end[1]), z=0.04),
            ]
            marker_array.markers.append(marker)

        self.map_marker_publisher.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfLineLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
