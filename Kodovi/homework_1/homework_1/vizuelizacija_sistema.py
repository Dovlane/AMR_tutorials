import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float64, Int64

class VizuelizacijaSistema(Node):

    def __init__(self):
        super().__init__('vizuelizacija_sistema')
        
        self.temperatura = None
        self.vlaznost = None
        self.osvetljenje = None

        self.temperatura_sub = self.create_subscription(
            Float64, 
            '/kuca/temperatura', 
            lambda msg: self.sensor_callback('/kuca/temperatura', msg), 
            10)
        
        self.vlaznost_sub = self.create_subscription(
            Float64, 
            '/kuca/vlaznost', 
            lambda msg: self.sensor_callback('/kuca/vlaznost', msg), 
            10)
        
        self.osvetljenje_sub = self.create_subscription(
            Int64, 
            '/kuca/osvetljenje', 
            lambda msg: self.sensor_callback('/kuca/osvetljenje', msg), 
            10)
        
        self.timer = self.create_timer(1.0, self.on_timer)

    def sensor_callback(self, topic_name, msg):
        if topic_name == '/kuca/temperatura':
            self.temperatura = msg.data
        elif topic_name == '/kuca/vlaznost':
            self.vlaznost = msg.data
        elif topic_name == '/kuca/osvetljenje':
            self.osvetljenje = msg.data

    def on_timer(self):
        if None in (self.temperatura, self.vlaznost, self.osvetljenje):
            self.get_logger().info('Cekam podatke sa svih senzora...')
            return

        self.get_logger().info(
            '[Nadzorna tabla] Temperatura: %.1f | Vlaznost: %.1f | Osvetljenje: %d \n'
            % (self.temperatura, self.vlaznost, self.osvetljenje)
        )


def main():
    rclpy.init() 
    node = VizuelizacijaSistema() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
