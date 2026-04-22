import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class Brava(Node):

    def __init__(self):
        super().__init__('brava')
        self.zakljucana = False
        self.service = self.create_service(
            SetBool,
            '/kuca/brava/zakljucaj',
            self.handle_zakljucaj_request,
        )
        self.get_logger().info('Servis brave je spreman na /kuca/brava/zakljucaj.')

    def handle_zakljucaj_request(self, request, response):
        self.zakljucana = request.data
        response.success = True

        if self.zakljucana:
            response.message = 'Vrata su zakljucana.'
        else:
            response.message = 'Vrata su otkljucana.'

        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = Brava()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
