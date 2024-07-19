import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OutboundPublisher(Node):
    def __init__(self):
        super().__init__("outbound_publisher")
        self.publisher = self.create_publisher(String, "outbound", 10)
        
    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing message: {message}")

def init():
    rclpy.init()
    return OutboundPublisher()
