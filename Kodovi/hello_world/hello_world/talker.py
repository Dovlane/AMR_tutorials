import rclpy 
from rclpy.node import Node 

from std_msgs.msg import String 

 
class Talker(Node):   

    def __init__(self): 
        super().__init__('talker') 
        self.publisher = self.create_publisher(String, 'chatter', 10) 
        self.timer = self.create_timer(1.0, self.on_timer) 

    def on_timer(self): 
        msg = String()  
        msg.data = 'Hello ROS 2'
        self.get_logger().info('I said: "%s"' %msg.data)
        self.publisher.publish(msg) 

def main():
    rclpy.init() 
    node = Talker() 
    rclpy.spin(node) 
