import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped, TaskResult
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header, String
import time
import requests
import queue

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        self.robot_id = '1'
        self.current_robot_id = None
        self.current_section = None
        self.is_navigating = False
        self.path_queue = queue.Queue()
        
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.id_subscription = self.create_subscription(String, 'robot_id', self.id_callback, 10)
        self.section_subscription = self.create_subscription(String, 'section', self.section_callback, 10)
        self.subscription = self.create_subscription(Path, 'planned_path_1', self.path_callback, 10)
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        
        self.publish_initial_pose()
        
        time.sleep(3)

    def publish_initial_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.2
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        pose_msg.pose.covariance = [0.0] * 36
        
        self.initial_pose_publisher.publish(pose_msg)
        self.get_logger().info("Published initial pose")
    
    def section_callback(self, msg):
        self.current_section = msg.data
        self.get_logger().info(f"Section updated : {self.current_section}")
        
        while not self.path_queue.empty():
            path_msg = self.path_queue.get()
            self.process_path(path_msg)
    
    def id_callback(self, msg):
        self.current_robot_id = msg.data
        self.get_logger().info(f"Robot ID updated: {self.current_robot_id}")
    
    
    def path_callback(self, msg):
        if self.current_section is None:
            self.get_logger().info("Section not update")
            self.path_queue.put(msg)
            return
        
        self.process_path(msg)
    
    def process_path(self, msg):
        self.get_logger().info("Path callback triggered")
        
        time.sleep(1)
        
        # if self.current_robot_id != self.robot_id:
        #     self.get_logger().info(f"Robot ID mismatch: {self.current_robot_id} != {self.robot_id}") 
        #     return
        
        # time.sleep(2)
        
        if self.is_navigating:
            self.get_logger().info("Already navigation, new path ignored")
            return
        
        self.is_navigating = True
        
        time.sleep(2)
        
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
        try:
            server_url = "http://192.168.1.100:5000/task/update"
            
            print(self.current_section)
            
            if self.current_section == "포장소":
                status = "PACKING"
            elif self.current_section in ["출고소 1", "출고소 2"]:
                status = "UNLOADING"
            elif self.current_section in ["집품 1-1", "집품 1-2", "집품 2-1", "집품 2-2", "집품 3-1", "집품 3-2", "집품 4-1", "집품 4-2"]:
                status = "PICKING"
            elif self.current_section in ["충전소 1", "충전소 2", "충전소 3", "충전소 4"]:
                status = "CHARGING"
            else:
                status = "UNKNOWN"
                
            
            data = {"robot_id": 1, "status": status}
            response = requests.post(server_url, json=data)
            if response.status_code == 200:
                self.get_logger().info("Successfully sent message to the server")
            else:
                self.get_logger().info(f"Failed to send message to the server: {response.status_code}")
                
        except Exception as e:
            self.get_logger().error(f"Exception while sending message to the server: {e}")
            pass 
            

                        
        # try:
        #     server_url = "http://192.168.1.100:5000/task/update"
            
        #     print(self.current_section)
            
        #     if self.current_section == "포장소":
        #         status = "PACKING"
        #     elif self.current_section in ["출고소 1", "출고소 2"]:
        #         status = "UNLOADING"
        #     elif self.current_section in ["집품 1-1", "집품 1-2", "집품 2-1", "집품 2-2", "집품 3-1", "집품 3-2", "집품 4-1", "집품 4-2"]:
        #         status = "PICKING"
        #     elif self.current_section in ["충전소 1", "충전소 2", "충전소 3", "충전소 4"]:
        #         status = "CHARGING"
        #     else:
        #         status = "UNKNOWN"
                
            
        #     data = {"robot_id": 1, "status": status}
        #     response = requests.post(server_url, json=data)
        #     if response.status_code == 200:
        #         self.get_logger().info("Successfully sent message to the server")
        #     else:
        #         self.get_logger().info(f"Failed to send message to the server: {response.status_code}")
                
        # except Exception as e:
        #     self.get_logger().error(f"Exception while sending message to the server: {e}")
        #     pass 

        # # 최종 결과 확인
        # result = self.nav.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     self.get_logger().info("Reached the destination successfully!")
                            
            
        # else:
        #     self.get_logger().info("Failed to reach the destination.")
            
        self.is_navigating = False
        
def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

