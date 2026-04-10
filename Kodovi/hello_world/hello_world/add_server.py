import rclpy
from rclpy.node import Node
from hello_world_interfaces.srv import AddInt

class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        self.srv = self.create_service(AddInt, 'add_int', self.callback)

    def callback(self, request, response):
        self.get_logger().info(f"Request: {request.a}")
        response.success = True
        return response

def main():
    rclpy.init()
    node = AddServer()
    rclpy.spin(node)
    rclpy.shutdown()
