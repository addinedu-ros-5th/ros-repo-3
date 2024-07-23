from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'class'))
from Astar import AStarPlanner
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped, TaskResult
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
import time

class AStarMovement(Node):
    def __init__(self):
        super().__init__('a_star_path_planning')
        self.publisher_ = self.create_publisher(String, 'astar_commands', 10)
        self.subscription = self.create_subscription(String, 'astar_commands', self.listener_callback, 10)
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.targets = {
            '집품 1-1': {'x': 1.185, 'y': 1.456, 'z': -0.003, 'w': 0.999},
            '집품 1-2': {'x': 1.15, 'y': 1.186, 'z': 0.027, 'w': 0.999},
            '집품 2-1': {'x': 1.325, 'y': 0.709, 'z': 0.046, 'w': 0.999},
            '집품 2-2': {'x': 1.250, 'y': 0.438, 'z': -0.011, 'w': 0.999},
            '집품 3-1': {'x': 0.305, 'y': 1.366, 'z': 0.997, 'w': 0.073},
            '집품 3-2': {'x': 0.427, 'y': 1.265, 'z': 0.998, 'w': 0.061},
            '집품 4-1': {'x': 0.216, 'y': 0.746, 'z': 0.999, 'w': 0.045},
            '집품 4-2': {'x': 0.288, 'y': 0.473, 'z': 0.999, 'w': 0.046},
            '포장소': {'x': 0.719, 'y': -0.586, 'z': -0.665, 'w': 0.747},
            '출고소 1': {'x': 1.328, 'y': -1.581, 'z': -0.668, 'w': 0.743},
            '출고소 2': {'x': 0.132, 'y': -1.761, 'z': -0.656, 'w': 0.754},
            '충전소': {'x': 0.000, 'y': 0.000, 'z': 0.000, 'w': 0.999}
        }
    
    def listener_callback(self, msg):
        target_name = msg.data
        if target_name in self.targets:
            self.move_to_target(target_name)
        else:
            self.get_logger().info(f"Target {target_name} is not recognized.")
    
    def move_to_target(self, target_name):
        target_pose = self.targets[target_name]
        self.get_logger().info(f"Moving to {target_name} at position ({target_pose['x']}, {target_pose['y']})...")

        # AStarPlanner 인스턴스 생성
        a_star = AStarPlanner(resolution=1, rr=1, padding=3)

        # 시작 좌표와 목표 좌표 설정
        sx_real, sy_real = 0.0, 0.0  # 시작 지점 좌표
        gx_real, gy_real = target_pose['x'], target_pose['y']  # 목표 지점 좌표

        # 경로 계획
        tpx, tpy = a_star.planning(sx_real, sy_real, gx_real, gy_real)

        # 경로 생성
        path = []
        for x, y in zip(tpx, tpy):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.nav.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = Quaternion(w=1.0)
            path.append(pose)

        # 경로 따라가기
        self.nav.followWaypoints(path)

        # 경로 완료까지 대기
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback:
                current_waypoint_index = feedback.current_waypoint
                if current_waypoint_index < len(path):
                    current_pose = path[current_waypoint_index].pose.position
                    remaining_distance = math.hypot(
                        target_pose['x'] - current_pose.x,
                        target_pose['y'] - current_pose.y
                    )
                    self.get_logger().info(f"Distance remaining: {remaining_distance:.2f} meters")
                    if remaining_distance <= 0.02:
                        self.get_logger().info("Arrived at the Target")
                        break

        # 최종 결과 확인
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {target_name} successfully!")
        else:
            self.get_logger().info(f"Failed to reach {target_name}.")

def main(args=None):
    rclpy.init(args=args)
    node = AStarMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
