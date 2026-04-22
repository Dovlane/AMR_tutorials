import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from random import uniform


class SenzorTemperatura(Node):

    def __init__(self):
        super().__init__('senzor_temperatura')
        self.publisher = self.create_publisher(Float64, '/kuca/temperatura', 10)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg = Float64()
        msg.data = uniform(18.0, 30.0)
        self.get_logger().info('Kucna temperatura: "%.1f" C' % msg.data)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SenzorTemperatura()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
