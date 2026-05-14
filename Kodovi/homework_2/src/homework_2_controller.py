#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from homework_2.srv import RobotCommand


MODE_MANUAL = 1
MODE_AUTO = 2
MODE_STOP = 3

MANUAL_FORWARD = 1
MANUAL_BACKWARD = 2
MANUAL_LEFT = 3
MANUAL_RIGHT = 4
MANUAL_STOP = 5

CONTROLLER_BASIC = 1
CONTROLLER_REVERSE = 2
CONTROLLER_CONSTANT_SPEED = 3


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class AutoGoal:
    x: float
    y: float
    controller_type: int
    linear_speed: float
    angular_speed: float


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_abs(value: float, limit: float) -> float:
    return clamp(value, -abs(limit), abs(limit))


def compute_polar_errors(robot_yaw: float, dx: float, dy: float) -> tuple[float, float]:
    """Return alpha and beta from the polar controller in AMR chapter 3.6."""
    heading_to_goal = math.atan2(dy, dx)
    alpha = normalize_angle(heading_to_goal - robot_yaw)
    beta = normalize_angle(-robot_yaw - alpha)
    return alpha, beta


class Homework2Controller(Node):
    def __init__(self) -> None:
        super().__init__("homework_2_controller")

        self.k_rho = 0.8
        self.k_alpha = 2.8
        self.k_beta = -0.6
        self.position_tolerance = 0.05
        self.manual_linear_speed = 0.15
        self.manual_angular_speed = 0.8
        self.max_linear_speed = 0.22
        self.max_angular_speed = 2.0
        self.manual_timeout = 0.6
        self.control_period = 0.05

        self.pose = RobotPose()
        self.has_odometry = False
        self.mode = MODE_STOP
        self.manual_twist = Twist()
        self.last_manual_command_time = 0.0
        self.auto_goal: Optional[AutoGoal] = None

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odometry_callback,
            10,
        )
        self.service = self.create_service(
            RobotCommand,
            "/robot_command_service",
            self.command_callback,
        )
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            "Homework 2 controller ready: service /robot_command_service, "
            "topic /cmd_vel, topic /odom."
        )

    def odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )

        self.pose = RobotPose(
            x=position.x,
            y=position.y,
            yaw=math.atan2(siny_cosp, cosy_cosp),
        )
        self.has_odometry = True

    def command_callback(
        self,
        request: RobotCommand.Request,
        response: RobotCommand.Response,
    ) -> RobotCommand.Response:
        if request.mode == MODE_STOP:
            self.stop_robot()
            response.success = True
            response.message = "Robot stopped."
            return response

        if request.mode == MODE_MANUAL:
            return self.handle_manual_request(request, response)

        if request.mode == MODE_AUTO:
            return self.handle_auto_request(request, response)

        response.success = False
        response.message = f"Unknown mode: {request.mode}."
        return response

    def handle_manual_request(
        self,
        request: RobotCommand.Request,
        response: RobotCommand.Response,
    ) -> RobotCommand.Response:
        linear_speed = (
            abs(request.linear_speed)
            if request.linear_speed > 0.0
            else self.manual_linear_speed
        )
        angular_speed = (
            abs(request.angular_speed)
            if request.angular_speed > 0.0
            else self.manual_angular_speed
        )
        linear_speed = min(linear_speed, self.max_linear_speed)
        angular_speed = min(angular_speed, self.max_angular_speed)

        command = request.manual_command
        twist = Twist()

        if command == MANUAL_FORWARD:
            twist.linear.x = linear_speed
            message = "Manual command: forward."
        elif command == MANUAL_BACKWARD:
            twist.linear.x = -linear_speed
            message = "Manual command: backward."
        elif command == MANUAL_LEFT:
            twist.angular.z = angular_speed
            message = "Manual command: rotate left."
        elif command == MANUAL_RIGHT:
            twist.angular.z = -angular_speed
            message = "Manual command: rotate right."
        elif command == MANUAL_STOP:
            self.stop_robot()
            response.success = True
            response.message = "Manual command: stop."
            return response
        else:
            response.success = False
            response.message = f"Unknown manual command: {command}."
            return response

        self.mode = MODE_MANUAL
        self.auto_goal = None
        self.manual_twist = twist
        self.last_manual_command_time = time.monotonic()

        response.success = True
        response.message = message
        return response

    def handle_auto_request(
        self,
        request: RobotCommand.Request,
        response: RobotCommand.Response,
    ) -> RobotCommand.Response:
        if not self.has_odometry:
            response.success = False
            response.message = "Odometry has not been received yet."
            return response

        if request.controller_type not in (
            CONTROLLER_BASIC,
            CONTROLLER_REVERSE,
            CONTROLLER_CONSTANT_SPEED,
        ):
            response.success = False
            response.message = f"Unknown controller type: {request.controller_type}."
            return response

        if not math.isfinite(request.goal_x) or not math.isfinite(request.goal_y):
            response.success = False
            response.message = "Goal coordinates must be finite numbers."
            return response

        linear_speed = (
            abs(request.linear_speed)
            if request.linear_speed > 0.0
            else self.max_linear_speed
        )
        angular_speed = (
            abs(request.angular_speed)
            if request.angular_speed > 0.0
            else self.max_angular_speed
        )

        self.auto_goal = AutoGoal(
            x=request.goal_x,
            y=request.goal_y,
            controller_type=request.controller_type,
            linear_speed=min(linear_speed, self.max_linear_speed),
            angular_speed=min(angular_speed, self.max_angular_speed),
        )
        self.mode = MODE_AUTO

        response.success = True
        response.message = (
            f"Automatic goal accepted: x={request.goal_x:.3f}, "
            f"y={request.goal_y:.3f}, controller={request.controller_type}."
        )
        return response

    def control_loop(self) -> None:
        if self.mode == MODE_MANUAL:
            if time.monotonic() - self.last_manual_command_time > self.manual_timeout:
                self.stop_robot()
                self.get_logger().warning("Manual command timeout. Robot stopped.")
                return

            self.publisher.publish(self.manual_twist)
            return

        if self.mode == MODE_AUTO and self.auto_goal is not None:
            self.run_auto_controller()

    def run_auto_controller(self) -> None:
        goal = self.auto_goal
        if goal is None:
            return

        dx = goal.x - self.pose.x
        dy = goal.y - self.pose.y
        rho = math.hypot(dx, dy)

        if rho <= self.position_tolerance:
            self.stop_robot()
            self.get_logger().info(
                f"Goal reached: x={goal.x:.3f}, y={goal.y:.3f}."
            )
            return

        twist = self.compute_auto_twist(goal, dx, dy, rho)
        self.publisher.publish(twist)

    def compute_auto_twist(
        self,
        goal: AutoGoal,
        dx: float,
        dy: float,
        rho: float,
    ) -> Twist:
        alpha, beta = compute_polar_errors(self.pose.yaw, dx, dy)
        direction = 1.0

        if goal.controller_type in (
            CONTROLLER_REVERSE,
            CONTROLLER_CONSTANT_SPEED,
        ) and abs(alpha) > math.pi / 2.0:
            direction = -1.0
            alpha = normalize_angle(alpha - math.copysign(math.pi, alpha))

        linear_velocity = direction * self.k_rho * rho
        angular_velocity = self.k_alpha * alpha + self.k_beta * beta

        linear_velocity = clamp_abs(linear_velocity, goal.linear_speed)
        angular_velocity = clamp_abs(angular_velocity, goal.angular_speed)

        if goal.controller_type == CONTROLLER_CONSTANT_SPEED:
            linear_velocity, angular_velocity = self.scale_to_constant_speed(
                linear_velocity,
                angular_velocity,
                goal.linear_speed,
            )

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        return twist

    def scale_to_constant_speed(
        self,
        linear_velocity: float,
        angular_velocity: float,
        target_speed: float,
    ) -> tuple[float, float]:
        if abs(linear_velocity) < 1e-6:
            return linear_velocity, angular_velocity

        scale = target_speed / abs(linear_velocity)
        return linear_velocity * scale, angular_velocity * scale

    def stop_robot(self) -> None:
        self.mode = MODE_STOP
        self.auto_goal = None
        self.manual_twist = Twist()
        self.publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Homework2Controller()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
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
