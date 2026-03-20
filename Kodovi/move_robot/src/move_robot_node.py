#!/usr/bin/env python3

import math
import threading

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from move_robot.srv import MoveRobotService


class MoveRobotServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("move_robot_service")
        self.callback_group = ReentrantCallbackGroup()
        self.robot_pose = Pose()
        self.initialized_pose = False
        self.target_distance = 1.0
        self.start_pose = None
        self.active_request = None
        self.motion_done = threading.Event()

        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription = self.create_subscription(
            Odometry,
            "odom",
            self.odometry_callback,
            10,
            callback_group=self.callback_group,
        )
        self.service = self.create_service(
            MoveRobotService,
            "move_robot_service",
            self.move_robot_go,
            callback_group=self.callback_group,
        )
        self.control_timer = self.create_timer(
            0.1,
            self.control_loop,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Publisher, subscriber, and service are ready.")

    def odometry_callback(self, data_odom: Odometry) -> None:
        self.robot_pose = data_odom.pose.pose
        self.initialized_pose = True

    def move_robot_go(
        self,
        request: MoveRobotService.Request,
        response: MoveRobotService.Response,
    ) -> MoveRobotService.Response:
        if not self.initialized_pose:
            self.get_logger().warning("Odometry has not been received yet.")
            response.move_done = False
            return response

        if self.active_request is not None:
            self.get_logger().warning("A motion request is already running.")
            response.move_done = False
            return response

        self.start_pose = Pose()
        self.start_pose.position.x = self.robot_pose.position.x
        self.start_pose.position.y = self.robot_pose.position.y
        self.active_request = {
            "x_speed": request.x_speed,
            "angular_speed": request.angular_speed,
        }
        self.motion_done.clear()
        while rclpy.ok() and not self.motion_done.wait(0.1):
            pass
        response.move_done = True
        self.get_logger().info("Motion request completed.")
        return response

    def control_loop(self) -> None:
        if self.active_request is None or self.start_pose is None:
            return

        distance = math.sqrt(
            math.pow(self.robot_pose.position.x - self.start_pose.position.x, 2)
            + math.pow(self.robot_pose.position.y - self.start_pose.position.y, 2)
        )

        if distance <= self.target_distance:
            pub_cmd = Twist()
            pub_cmd.linear.x = self.active_request["x_speed"]
            pub_cmd.angular.z = self.active_request["angular_speed"]
            self.publisher.publish(pub_cmd)
            return

        self.publisher.publish(Twist())
        self.active_request = None
        self.start_pose = None
        self.motion_done.set()
        self.get_logger().info("Target distance reached. Robot stopped.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveRobotServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
