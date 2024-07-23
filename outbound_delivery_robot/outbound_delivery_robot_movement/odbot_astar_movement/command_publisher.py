import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'movement', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = '집품 1-1'
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()