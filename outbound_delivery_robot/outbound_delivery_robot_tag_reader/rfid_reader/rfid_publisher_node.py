import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO

class RFIDPublisher(Node):
    def __init__(self):
        super().__init__('rfid_publisher_node')
        self.publisher_ = self.create_publisher(String, 'rfid_data', 10)
        self.reader = SimpleMFRC522()
        self.timer = self.create_timer(1.0, self.publish_rfid_data)

    def publish_rfid_data(self):
        try:
            id, text = self.reader.read()
            self.get_logger().info(f'Read RFID id: {id}, text: {text}')
            msg = String()
            msg.data = f'ID: {id}, Text: {text}'
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading RFID: {e}')

def main(args=None):
    rclpy.init(args=args)
    rfid_publisher = RFIDPublisher()
    try:
        rclpy.spin(rfid_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rfid_publisher.destroy_node()
        GPIO.cleanup()  # 프로그램 종료 시 GPIO.cleanup() 호출
        rclpy.shutdown()

if __name__ == '__main__':
    main()
