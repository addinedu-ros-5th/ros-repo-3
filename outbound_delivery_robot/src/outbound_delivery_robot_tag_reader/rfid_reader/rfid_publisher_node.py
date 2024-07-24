import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO
import requests

class RFIDPublisher(Node):
    def __init__(self):
        super().__init__('rfid_publisher_node')
        self.publisher_ = self.create_publisher(String, 'rfid_data', 10)
        self.reader = SimpleMFRC522()
        self.timer = self.create_timer(1.0, self.publish_rfid_data)
        self.server_url = "http://192.168.0.79:5000/product/tag"

    def publish_rfid_data(self):
        try:
            id, text = self.reader.read()
            self.get_logger().info(f'Read RFID id: {id}, text: {text}')
            msg = String()
            msg.data = f'ID: {id}, Text: {text}'
            self.publisher_.publish(msg)
            
            self.send_request_to_server(id, text)
        except Exception as e:
            self.get_logger().error(f'Error reading RFID: {e}')
            
    def send_request_to_server(self, id, text):
        try:
            payload = {'product' : text.strip()}
            response = requests.post(self.server_url, params=payload)
            if response.status_code == 200:
                self.get_logger().info(f"Successfully sent data to server: {response.status_code}")
                
            else:
                self.get_logger().error(f'Failed to send data to server : {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f"Error sending request to server: {e}")

def main(args=None):
    rclpy.init(args=args)
    rfid_publisher = RFIDPublisher()
    try:
        rclpy.spin(rfid_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rfid_publisher.destroy_node()
        GPIO.cleanup() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
