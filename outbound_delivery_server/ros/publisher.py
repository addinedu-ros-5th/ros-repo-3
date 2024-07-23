from rclpy.node import Node
from outbound_delivery_robot_interfaces.msg import Location
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__("publisher")
        self.loc_publisher = self.create_publisher(Location, "location", 10)
        self.detect_publisher = self.create_publisher(String, "detection", 10)
        
    def publish_location(self, location):
        msg = Location()
        msg.section = location.section
        msg.x = location.x
        msg.y = location.y
        msg.z = location.z
        msg.x = location.x
        self.loc_publisher.publish(msg)
        
    def publish_path(self, path):
        pass
    
    def publish_detection(self, detection):
        msg = String()
        msg.data = detection
        self.detect_publisher.publish(msg)
        

def pub():
    return Publisher()
