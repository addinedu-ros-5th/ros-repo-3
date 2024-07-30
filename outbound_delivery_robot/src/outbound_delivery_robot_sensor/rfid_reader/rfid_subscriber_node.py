import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RFIDSubscriber(Node):
    def __init__(self):
        super().__init__('rfid_subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'rfid_data',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    rfid_subscriber = RFIDSubscriber()
    try:
        rclpy.spin(rfid_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rfid_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
