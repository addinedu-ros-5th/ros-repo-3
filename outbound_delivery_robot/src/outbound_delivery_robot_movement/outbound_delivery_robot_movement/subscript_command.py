import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        self.publisher_ = self.create_publisher(String, 'astar_commands', 10)
        self.subscription = self.create_subscription(
            String,
            'robot_1',  # 'robot_1'을 'astar_commands'로 변경
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        target_name = msg.data
        message = String()
        message.data = target_name
        self.publisher_.publish(message)
        self.get_logger().info(f"Received command: {target_name}")

        # 여기서 명령에 따라 로봇을 이동시키는 코드 추가
        # 예: move_robot_to_target(target_name)

def main(args=None):
    rclpy.init(args=args)
    command_subscriber = CommandSubscriber()
    rclpy.spin(command_subscriber)
    command_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()