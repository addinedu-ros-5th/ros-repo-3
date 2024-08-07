import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImgSubscriber(Node):
    def __init__(self):
        super().__init__('img_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera',
            self.listener_callback,
            10)
        self.cv_bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Camera", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main():
    rclpy.init()
    node = ImgSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
