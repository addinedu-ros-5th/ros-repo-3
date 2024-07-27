import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from Class.Astar import AStarPlanner
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped, TaskResult
from outbound_delivery_robot_interfaces.msg import Location

class AStarMovement(Node):
    def __init__(self):
        super().__init__('a_star_path_planning')
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)
        self.id_publisher = self.create_publisher(String, 'robot_id', 10)
        self.location_subscription = self.create_subscription(Location, 'location', self.location_callback, 10)
        # Ensure the subscription is not garbage collected
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        self.get_logger().info("Parameter callback triggered")
        return rclpy.node.Node.CallbackReturn.SUCCESS

    def location_callback(self, msg):
        self.get_logger().info(f"Location callback triggered")
        self.get_logger().info(f"Received location: robot ID = {msg.robot_id}, section={msg.section}, x={msg.x}, y={msg.y}, z={msg.z}, w={msg.w}")
        self.move_to_target(msg)
    
    def move_to_target(self, location):
        target_pose = {
            'x': location.x,
            'y': location.y,
            'z': location.z,
            'w': location.w
        }
        self.get_logger().info(f"Moving to {location.section} at position ({target_pose['x']}, {target_pose['y']})...")
        
        a_star = AStarPlanner(resolution=1, rr=0.3, padding=3)
        
        sx_real, sy_real = 0.0, 0.0
        gx_real, gy_real = target_pose['x'], target_pose['y']
        
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
        
        # Example: If you want to send both robot ID and path in a single message
        # Create a new message type if needed
        # For simplicity, we're just publishing the robot ID here
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

