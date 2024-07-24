import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'astar_commands', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.timer_cancelled = False  # 타이머가 취소되었는지 추적

    def timer_callback(self):
        if not self.timer_cancelled:  # 타이머가 취소되지 않았다면
            msg = String()
            msg.data = '집품 1-1'
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published command: {msg.data}")
            self.timer.cancel()  # 타이머 취소
            self.timer_cancelled = True

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
