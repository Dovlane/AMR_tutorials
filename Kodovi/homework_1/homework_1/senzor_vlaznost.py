import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from random import uniform


class SenzorVlaznost(Node):

    def __init__(self):
        super().__init__('senzor_vlaznost')
        self.publisher = self.create_publisher(Float64, '/kuca/vlaznost', 10)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg = Float64()
        msg.data = uniform(30.0, 70.0)
        self.get_logger().info('Vlaznost u kuci: "%.1f" %%' % msg.data)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SenzorVlaznost()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
