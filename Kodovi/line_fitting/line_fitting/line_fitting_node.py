#!/usr/bin/env python3

import numpy
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node


class LineFittingNode(Node):
    def __init__(self) -> None:
        super().__init__("line_fitting_node")
        self.subscription = self.create_subscription(
            LaserScan,
            "/kobuki/laser/scan",
            self.callback,
            10,
        )
        self.get_logger().info("Ready to fit lines!")

    def callback(self, data: LaserScan) -> None:
        rho = data.ranges[0:10]
        if len(rho) < 10:
            self.get_logger().warning("Received fewer than 10 scan samples.")
            return

        theta = []
        for i in range(10):
            theta.append(data.angle_min + i * data.angle_increment)

        x = numpy.zeros(10)
        y = numpy.zeros(10)
        for i in range(10):
            x[i] = rho[i] * numpy.cos(theta[i])
            y[i] = rho[i] * numpy.sin(theta[i])

        xc = numpy.mean(x)
        yc = numpy.mean(y)

        x_t = xc - x
        y_t = yc - y

        sum1 = numpy.sum(x_t * y_t)
        sum2 = numpy.sum(y_t**2 - x_t**2)

        alpha = 0.5 * numpy.arctan2(-2 * sum1, sum2)
        r = xc * numpy.cos(alpha) + yc * numpy.sin(alpha)

        self.get_logger().info(f"r={r}")
        self.get_logger().info(f"alpha={alpha}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFittingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
