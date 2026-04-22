import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from random import randint


class SenzorOsvetljenje(Node):

    def __init__(self):
        super().__init__('senzor_osvetljenje')
        self.publisher = self.create_publisher(Int64, '/kuca/osvetljenje', 10)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg = Int64()
        msg.data = randint(0, 1000)
        self.get_logger().info('Vlaznost u kuci: "%d"lx', msg.data)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SenzorOsvetljenje()
    rclpy.spin(node)
