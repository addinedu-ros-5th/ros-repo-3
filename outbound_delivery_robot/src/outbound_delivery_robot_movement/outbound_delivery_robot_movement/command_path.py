import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped, TaskResult
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

    def path_callback(self, msg):
        self.get_logger().info(f"Received path with {len(msg.poses)} points.")

        # 경로 따라가기
        self.nav.followWaypoints(msg.poses)

        # 경로 완료까지 대기
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback:
                current_waypoint_index = feedback.current_waypoint
                if current_waypoint_index < len(msg.poses):
                    current_pose = msg.poses[current_waypoint_index].pose.position
                    remaining_distance = math.hypot(
                        msg.poses[-1].pose.position.x - current_pose.x,
                        msg.poses[-1].pose.position.y - current_pose.y
                    )
                    self.get_logger().info(f"Distance remaining: {remaining_distance:.2f} meters")
                    if remaining_distance <= 0.02:
                        self.get_logger().info("Arrived at the Target")
                        break

        # 최종 결과 확인
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Reached the destination successfully!")
        else:
            self.get_logger().info("Failed to reach the destination.")
        
def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()