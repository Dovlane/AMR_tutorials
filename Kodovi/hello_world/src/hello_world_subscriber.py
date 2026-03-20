#!/usr/bin/env python3

from std_msgs.msg import String

import rclpy
from rclpy.node import Node


class HelloWorldSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("hello_world_subscriber")
        self.subscription = self.create_subscription(
            String,
            "hello_topic",
            self.callback,
            10,
        )

    def callback(self, data: String) -> None:
        self.get_logger().info(f"I heard {data.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HelloWorldSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
