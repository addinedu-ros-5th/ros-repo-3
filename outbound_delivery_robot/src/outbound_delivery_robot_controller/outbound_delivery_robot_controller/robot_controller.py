import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

from std_msgs.msg import String
from outbound_delivery_robot_interfaces.msg import TaskRequest, AStar
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist

from ament_index_python.packages import get_package_share_directory

import os
from math import pi, sqrt, atan2
from tf_transformations import euler_from_quaternion

bt_file = os.path.join(get_package_share_directory('outbound_delivery_robot_behavior'), 'behavior_tree', 'bt.xml')

global amcl
amcl = PoseWithCovarianceStamped()

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

class TaskSubscriber(Node):
    def __init__(self):
        super().__init__('send_task_subscriber')

        self.subscription = self.create_subscription(
            TaskRequest,
            'task',
            self.listener_callback,
        10)

    def listener_callback(self, msg):
        print(msg)

class GoPoseNode(Node):
    def __init__(self):
        super().__init__('go_pose_navigator')

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.amcl = PoseWithCovarianceStamped()
        self.navigator = BasicNavigator()

        self.vel_linear = 0.1
        self.vel_angle = 0.2

        self.done_publisher = self.create_publisher(String, '/done_task', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.astar_path_sub = self.create_subscription(AStar,
                                                       'astar_paths',
                                                       self.astarCallback,
                                                       10)

    def astarCallback(self, astar_paths):
        global amcl

        if astar_paths.length < 1:
            print("Invalid Paths")
            msg = String()
            msg.data = "REJECT"
            self.done_publisher.publish(msg)
            return
        
        i = 0
        while True:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = astar_paths.pose[i].position.x
            goal_pose.pose.position.y = astar_paths.pose[i].position.y
            goal_pose.pose.orientation.z = astar_paths.poses[i].orientation.z
            goal_pose.pose.orientation.w = astar_paths.poses[i].orientation.w
            self.navigator.goToPose(goal_pose, behavior_tree=bt_file)

            j = 0
            waiting_count = 0
            while not self.navigator.isTaskComplete():
                j += 1
                feedback = self.navigator.getFeedback()
                if feedback and j % 5 == 0:
                    estimated_time = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    print(f'Estimated time of arrival: {estimated_time:.0f} seconds.')

                    if estimated_time == 0 and waiting_count >= 10:
                        break
                    waiting_count = waiting_count + 1 if estimated_time == 0 else 0

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
                        self.navigator.cancelTask()

            twist_msg = Twist()
            diff_theta = self.calc_diff_theta(goal_pose.pose.position.x, goal_pose.pose.position.y)
            before_diff_theta = diff_theta

            while diff_theta > 0.01:
                diff_theta = self.calc_diff_theta(goal_pose.pose.position.x, goal_pose.pose.position.y)

                if before_diff_theta < diff_theta:
                    break

                twist_msg.angular.z = -self.vel_angle if diff_theta > (pi / 2) or diff_theta < -(pi / 2) else self.vel_angle
                self.cmd_vel_pub.publish(twist_msg)
                before_diff_theta = diff_theta

            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)

            diff_distance = self.calc_diff_distance(amcl.pose.pose.position.x, goal_pose.pose.position.x, amcl.pose.pose.position.y, goal_pose.pose.position.y)
            before_diff_distance = diff_distance

            while diff_distance > 0.01:
                diff_distance = self.calc_diff_distance(amcl.pose.pose.position.x, goal_pose.pose.position.x, amcl.pose.pose.position.y, goal_pose.pose.position.y)

                if before_diff_distance < diff_distance:
                    break

                twist_msg.linear.x = self.vel_linear
                self.cmd_vel_pub.publish(twist_msg)
                before_diff_distance = diff_distance

            twist_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(twist_msg)

            result = self.navigator.getResult()
            if diff_distance <= 0.03:
                self.navigator.cancelTask()
                if i == astar_paths.length - 1:
                    print('Goal succeeded!')
                    msg = String()
                    msg.data = "OK"
                    self.done_publisher.publish(msg)
                    break
                else:
                    i += 1
            elif result == TaskResult.SUCCEEDED:
                if i == astar_paths.length - 1:
                    print('Goal succeeded!')
                    msg = String()
                    msg.data = "OK"
                    self.done_publisher.publish(msg)
                    break
                else:
                    i += 1
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

    def calc_diff_distance(self, sx, gx, sy, gy):
        return sqrt(pow(gx - sx, 2) + pow(gy - sy, 2))

    def calc_diff_theta(self, gx, gy):
        alpha = euler_from_quaternion([0, 0, self.amcl.pose.pose.orientation.z, self.amcl.pose.pose.orientation.w])[2]
        beta = atan2((gy - self.amcl.pose.pose.position.y), (gx - self.amcl.pose.pose.position.x))
        return beta - alpha

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    send_task_subscriber = TaskSubscriber()
    executor.add_node(send_task_subscriber)

    go_pose_node = GoPoseNode()
    executor.add_node(go_pose_node)

    amcl_subscriber = AMCLSubscriber()
    executor.add_node(amcl_subscriber)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        send_task_subscriber.destroy_node()
        go_pose_node.destroy_node()
        amcl_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
