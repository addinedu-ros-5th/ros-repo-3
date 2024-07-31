import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from Class.Astar import AStarPlanner
from outbound_delivery_robot_interfaces.msg import Location
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import queue


class AStarMovement(Node):
    def __init__(self):
        super().__init__('a_star_path_planning')
        
        self.robot_initial_position = {
            1: (0, 0.2),
            2: (0, -0.1),
            3: (0, -0.4),
            4: (0, -0.7)
        }
        
        self.robot_current_position = {
            1: None,
            2: None,
            3: None,
            4: None
        }
        
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.id_publisher = self.create_publisher(String, 'robot_id', 10)
        
        self.robot_1_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.robot_1_pose_callback, 10)
        self.robot_2_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_1', self.robot_2_pose_callback, 10)
        self.robot_3_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_2', self.robot_3_pose_callback, 10)
        self.robot_4_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose_3', self.robot_4_pose_callback, 10)
        
        self.location_subscription = self.create_subscription(Location, 'location', self.location_callback, 10)
        
        
        
        self.add_on_set_parameters_callback(self.parameter_callback)


    def parameter_callback(self, params):
        self.get_logger().info("Parameter callback triggered")
        return rclpy.node.Node.CallbackReturn.SUCCESS

    
    def robot_4_pose_callback(self, msg):
        robot_id = 4
        self.robot_current_position[robot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Updated AMCL_position for robot {robot_id}: {self.robot_current_position[robot_id]}")
    
    def robot_3_pose_callback(self, msg):
        robot_id = 3
        self.robot_current_position[robot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Updated AMCL_position for robot {robot_id}: {self.robot_current_position[robot_id]}")
    
    def robot_2_pose_callback(self, msg):
        robot_id = 2
        self.robot_current_position[robot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Updated AMCL_position for robot {robot_id}: {self.robot_current_position[robot_id]}")
    
    def robot_1_pose_callback(self, msg):
        robot_id = 1
        self.robot_current_position[robot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Updated AMCL_position for robot {robot_id}: {self.robot_current_position[robot_id]}")

    def location_callback(self, msg):

        self.get_logger().info(f"Location callback triggered")
        self.get_logger().info(f"Received location: robot ID = {msg.robot_id}, section={msg.section}, x={msg.x}, y={msg.y}, z={msg.z}, w={msg.w}")
        self.move_to_target(msg)

    def move_to_target(self, location):
        
        robot_id = location.robot_id
        
        if self.robot_current_position[robot_id]:
            sx_real, sy_real = self.robot_current_position[robot_id]
        else:
            sx_real, sy_real = self.robot_initial_position[robot_id]
        

        gx_real = location.x
        gy_real = location.y
        
        sx_real = round(sx_real, 2)
        sy_real = round(sy_real, 2)
        gx_real = round(gx_real, 2)
        gy_real = round(gy_real, 2)


        self.get_logger().info(f"Moving to {location.section} from position ({sx_real}, {sy_real}) to ({gx_real}, {gy_real})")

        a_star = AStarPlanner(resolution=1, rr=0.5, padding=3)

        tpx, tpy = a_star.planning(sx_real, sy_real, gx_real, gy_real)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in zip(tpx, tpy):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = Quaternion(w=1.0)
            path.poses.append(pose)

        self.path_publisher.publish(path)
        self.get_logger().info(f"Path to {location.section} published")

        id_msg = String()
        id_msg.data = str(location.robot_id)
        self.id_publisher.publish(id_msg)
        self.get_logger().info(f'Robot ID {location.robot_id} published')

def main(args=None):
    rclpy.init(args=args)
    node = AStarMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

