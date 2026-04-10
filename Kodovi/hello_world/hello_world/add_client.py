import sys
import rclpy
from rclpy.node import Node
from hello_world_interfaces.srv import AddInt

class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.cli = self.create_client(AddInt, 'add_int')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        self.req = AddInt.Request()

    def send_request(self, a):
        self.req.a = a
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    node = AddClient()
    future = node.send_request(int(sys.argv[1]))
    rclpy.spin_until_future_complete(node, future)
    print(f"Success = {future.result().success}")
    node.destroy_node()
    rclpy.shutdown()
