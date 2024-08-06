import rclpy
from rclpy.node import Node
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO
import requests

class RFID(Node):
    def __init__(self):
        super().__init__('tag_reader')
        self.get_logger().info("tag_reader node is starting...")
        self.reader = SimpleMFRC522()
        self.robot_id = 1
        self.last_read_data = None
        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def read_rfid_tag(self):
        id, data = self.reader.read()
        return id
    
    def send_request_to_server(self, data):
        url = f"http://192.168.1.100:5000/tag/picking"
        self.get_logger().info(f"Tag data: {data}")
        json = {'robot_id': self.robot_id, 'product_id': data}
        try:
            response = requests.post(url, json=json)
            if response.status_code == 200:
                self.get_logger().info(f"Successfully sent data to server: {response.status_code}")
            else:
                self.get_logger().error(f"Failed to send data to server: {response.status_code}")
        except ConnectionError:
            pass
    
    def timer_callback(self):
        current_data = self.read_rfid_tag()
        self.get_logger().info(f"Current Id : {current_data}")
        if current_data != self.last_read_data:
            self.last_read_data = current_data
            self.send_request_to_server(current_data)

def main(args=None):
    rclpy.init(args=args)
    
    rfid = RFID()
    rclpy.spin(rfid)
    
    rfid.destroy_node()
    GPIO.cleanup() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
