#!/usr/bin/env python3

from std_msgs.msg import String

import rclpy
from rclpy.node import Node


class HelloWorldPublisher(Node):
    def __init__(self) -> None:
        super().__init__("hello_world_publisher")
        self.publisher_ = self.create_publisher(String, "hello_topic", 10)
        self.timer = self.create_timer(0.1, self.publish_message)

    def publish_message(self) -> None:
        data = String()
        data.data = f"Hello world {self.get_clock().now().nanoseconds / 1e9:.3f}"
        self.get_logger().info(data.data)
        self.publisher_.publish(data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HelloWorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
