#!/usr/bin/env python3

from pathlib import Path

import rclpy
from rclpy.node import Node

from hello_world.srv import AddValueFile


class AddValueFileService(Node):
    def __init__(self) -> None:
        super().__init__("add_value_file_service")
        self.output_dir = Path.home() / ".ros" / "hello_world"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.output_file = self.output_dir / "value_file.txt"
        self.service = self.create_service(
            AddValueFile,
            "add_value_file",
            self.response_callback,
        )
        self.get_logger().info(f"Service is ready. Writing values to {self.output_file}")

    def response_callback(
        self,
        request: AddValueFile.Request,
        response: AddValueFile.Response,
    ) -> AddValueFile.Response:
        with self.output_file.open("a", encoding="utf-8") as output_stream:
            output_stream.write(f"{request.value}\n")
        response.response = True
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AddValueFileService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
