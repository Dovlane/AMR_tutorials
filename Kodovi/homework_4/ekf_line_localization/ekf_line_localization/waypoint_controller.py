#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32
import yaml


@dataclass(frozen=True)
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


def default_share_path(*parts: str) -> str:
    try:
        base_path = Path(get_package_share_directory("ekf_line_localization"))
    except PackageNotFoundError:
        base_path = Path(__file__).resolve().parents[1]
    return str(base_path.joinpath(*parts))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_abs(value: float, limit: float) -> float:
    return clamp(value, -abs(limit), abs(limit))


def quaternion_to_yaw(orientation: object) -> float:
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def load_waypoints(path: str | Path) -> list[Waypoint]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    raw_waypoints = data.get("waypoints", data)
    waypoints = []
    for raw in raw_waypoints:
        if isinstance(raw, dict):
            x = raw["x"]
            y = raw["y"]
            yaw = raw.get("yaw", raw.get("theta", 0.0))
        else:
            if len(raw) == 2:
                x, y = raw
                yaw = 0.0
            else:
                x, y, yaw = raw[:3]
        waypoints.append(Waypoint(float(x), float(y), normalize_angle(float(yaw))))

    if not waypoints:
        raise ValueError(f"No waypoints found in {path}")
    return waypoints


class EkfWaypointController(Node):
    def __init__(self) -> None:
        super().__init__("ekf_waypoint_controller")

        default_waypoint_file = default_share_path("config", "waypoints.yaml")

        self.declare_parameter("waypoint_file", default_waypoint_file)
        self.declare_parameter("feedback_topic", "/ekf_pose")
        self.declare_parameter("feedback_type", "pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("waypoint_index_topic", "/waypoint_index")
        self.declare_parameter("control_period", 0.02)
        self.declare_parameter("k_rho", 0.8)
        self.declare_parameter("k_alpha", 2.8)
        self.declare_parameter("k_beta", -0.6)
        self.declare_parameter("k_yaw", 1.8)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("yaw_tolerance", 0.12)
        self.declare_parameter("max_linear_speed", 0.22)
        self.declare_parameter("max_angular_speed", 2.84)
        self.declare_parameter("reverse_enabled", True)
        self.declare_parameter("rate_limit_enabled", True)
        self.declare_parameter("max_linear_step", 0.025)
        self.declare_parameter("max_angular_step", 0.25)

        waypoint_file = str(self.get_parameter("waypoint_file").value)
        self.waypoints = load_waypoints(waypoint_file)

        self.pose = RobotPose()
        self.has_feedback = False
        self.current_waypoint_index = 0
        self.previous_twist = Twist()

        self.k_rho = float(self.get_parameter("k_rho").value)
        self.k_alpha = float(self.get_parameter("k_alpha").value)
        self.k_beta = float(self.get_parameter("k_beta").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        waypoint_index_topic = str(self.get_parameter("waypoint_index_topic").value)
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.index_publisher = self.create_publisher(Int32, waypoint_index_topic, 10)

        feedback_topic = str(self.get_parameter("feedback_topic").value)
        feedback_type = str(self.get_parameter("feedback_type").value).lower()
        if feedback_type == "odom":
            self.subscription = self.create_subscription(
                Odometry,
                feedback_topic,
                self.odom_callback,
                10,
            )
        else:
            self.subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                feedback_topic,
                self.pose_callback,
                10,
            )

        period = float(self.get_parameter("control_period").value)
        self.timer = self.create_timer(period, self.control_loop)

        self.publish_waypoint_index()
        self.get_logger().info(
            "Waypoint controller ready "
            f"(feedback={feedback_topic}, type={feedback_type}, "
            f"waypoints={len(self.waypoints)})."
        )

    def pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        self.pose = RobotPose(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=quaternion_to_yaw(pose.orientation),
        )
        self.has_feedback = True

    def odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.pose = RobotPose(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=quaternion_to_yaw(pose.orientation),
        )
        self.has_feedback = True

    def control_loop(self) -> None:
        if not self.has_feedback:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        twist, reached = self.compute_waypoint_twist(waypoint)

        if reached:
            self.get_logger().info(
                "Waypoint reached "
                f"{self.current_waypoint_index + 1}/{len(self.waypoints)}: "
                f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.2f}"
            )
            self.current_waypoint_index += 1
            self.publish_waypoint_index()
            if self.current_waypoint_index >= len(self.waypoints):
                self.stop_robot()
                self.get_logger().info("Waypoint mission complete.")
                return

            twist = Twist()

        twist = self.limit_twist(twist)
        self.previous_twist = twist
        self.publisher.publish(twist)

    def compute_waypoint_twist(self, waypoint: Waypoint) -> tuple[Twist, bool]:
        dx = waypoint.x - self.pose.x
        dy = waypoint.y - self.pose.y
        rho = math.hypot(dx, dy)
        yaw_error = normalize_angle(waypoint.yaw - self.pose.yaw)

        if rho <= self.position_tolerance:
            if abs(yaw_error) <= self.yaw_tolerance:
                return Twist(), True

            twist = Twist()
            twist.angular.z = self.k_yaw * yaw_error
            return twist, False

        heading_to_goal = math.atan2(dy, dx)
        alpha = normalize_angle(heading_to_goal - self.pose.yaw)
        direction = 1.0

        if bool(self.get_parameter("reverse_enabled").value) and abs(alpha) > math.pi / 2:
            direction = -1.0
            alpha = normalize_angle(alpha - math.copysign(math.pi, alpha))

        beta = normalize_angle(waypoint.yaw - self.pose.yaw - alpha)

        twist = Twist()
        twist.linear.x = direction * self.k_rho * rho
        twist.angular.z = self.k_alpha * alpha + self.k_beta * beta
        return twist, False

    def limit_twist(self, twist: Twist) -> Twist:
        limited = Twist()
        limited.linear.x = clamp_abs(twist.linear.x, self.max_linear_speed)
        limited.angular.z = clamp_abs(twist.angular.z, self.max_angular_speed)

        if not bool(self.get_parameter("rate_limit_enabled").value):
            return limited

        max_linear_step = abs(float(self.get_parameter("max_linear_step").value))
        max_angular_step = abs(float(self.get_parameter("max_angular_step").value))
        limited.linear.x = self.previous_twist.linear.x + clamp_abs(
            limited.linear.x - self.previous_twist.linear.x,
            max_linear_step,
        )
        limited.angular.z = self.previous_twist.angular.z + clamp_abs(
            limited.angular.z - self.previous_twist.angular.z,
            max_angular_step,
        )
        return limited

    def publish_waypoint_index(self) -> None:
        message = Int32()
        message.data = int(self.current_waypoint_index)
        self.index_publisher.publish(message)

    def stop_robot(self) -> None:
        self.previous_twist = Twist()
        self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfWaypointController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
