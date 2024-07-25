import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from ament_index_python.packages import get_package_share_directory
import os

bt_file = os.path.join(get_package_share_directory('outbound_delivery_robot_behavior'), 'behavior_tree', 'bt.xml')

class AMCLSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_subscriber')

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, amcl_pose_qos)

    def amcl_callback(self, msg):
        global amcl
        amcl = msg

class PersonDetectionSubscriber(Node):
    def __init__(self, controller):
        super().__init__('person_detection_subscriber')

        self.create_subscription(String, '/detection', self.person_detection_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.detected = False
        self.controller = controller

        self.create_timer(0.1, self.check_person_detection_status)

    def person_detection_callback(self, msg):
        if msg.data == "person":
            self.detected = True
            self.controller.stop_robot()
        elif msg.data == "robot":
            self.detected = True
            self.controller.avoid_obstacle()
        else:
            self.detected = False

    def check_person_detection_status(self):
        if not self.detected:
            self.controller.resume_robot()

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.navigator = BasicNavigator()
        self.person_detection_subscriber = PersonDetectionSubscriber(self)
        self.current_goal_pose = None

    def navigate_to_pose(self, pose):
        self.current_goal_pose = pose
        self.navigator.goToPose(pose, behavior_tree=bt_file)
    
    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.person_detection_subscriber.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Person detected: Robot stopped.')

    def resume_robot(self):
       if self.current_goal_pose:
            self.navigator.goToPose(self.current_goal_pose, behavior_tree=bt_file)
            self.get_logger().info('Resuming navigation to the goal pose.')

    def avoid_obstacle(self):
        self.get_logger().info('Avoiding obstacle!')

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.2
        self.person_detection_subscriber.cmd_vel_pub.publish(twist_msg)

        self.create_timer(2.0, self.resume_navigation_after_avoidance)

    def resume_navigation_after_avoidance(self):
        self.get_logger().info('Resuming navigation after obstacle avoidance.')
        if self.current_goal_pose:
            self.navigator.goToPose(self.current_goal_pose, behavior_tree=bt_file)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    robot_controller = RobotController()
    amcl_subscriber = AMCLSubscriber()

    executor.add_node(robot_controller)
    executor.add_node(amcl_subscriber)
    executor.add_node(robot_controller.person_detection_subscriber)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        robot_controller.destroy_node()
        amcl_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
