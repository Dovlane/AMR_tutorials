import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 

class Listener(Node): 

    def __init__(self): 
        super().__init__('listener') 
        self.subscription = self.create_subscription( 
              String,  
              'chatter', 
              self.listener_callback,  
              10) 
        self.subscription # prevent unused variable warning 

    def listener_callback(self, msg): 
        self.get_logger().info('I heard: "%s"' %msg.data)  

def main():
    rclpy.init() 
    node = Listener() 
    rclpy.spin(node) 
