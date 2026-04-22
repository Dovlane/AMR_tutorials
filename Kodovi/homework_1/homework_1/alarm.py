import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class Alarm(Node):

    def __init__(self):
        super().__init__('alarm')
        self.aktivan = True
        self.minimalna_temperatura = 15.0
        self.maksimalna_temperatura = 35.0

        self.service = self.create_service(
            SetBool,
            '/kuca/alarm/aktiviraj',
            self.handle_alarm_request,
        )
        self.temperatura_sub = self.create_subscription(
            Float64,
            '/kuca/temperatura',
            self.temperatura_callback,
            10,
        )

        self.get_logger().info('Alarm je spreman na /kuca/alarm/aktiviraj.')

    def handle_alarm_request(self, request, response):
        self.aktivan = request.data
        response.success = True

        if self.aktivan:
            response.message = 'Alarm je aktiviran.'
        else:
            response.message = 'Alarm je deaktiviran.'

        self.get_logger().info(response.message)
        return response

    def temperatura_callback(self, msg):
        temperatura = msg.data
        if not self.aktivan:
            return

        if temperatura < self.minimalna_temperatura or temperatura > self.maksimalna_temperatura:
            self.get_logger().warning(
                'UPOZORENJE: temperatura %.1f C je van dozvoljenog opsega [%.1f, %.1f].'
                % (
                    temperatura,
                    self.minimalna_temperatura,
                    self.maksimalna_temperatura,
               )
            )


def main():
    rclpy.init()
    node = Alarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
