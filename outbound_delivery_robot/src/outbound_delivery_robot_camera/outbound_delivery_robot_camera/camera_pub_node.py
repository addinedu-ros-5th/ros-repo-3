import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(CompressedImage, '/camera', 10)

        time_period = 0.1
        self.timer = self.create_timer(time_period, self.time_callback)

        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()
        self.msg = CompressedImage()

    def time_callback(self):
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (320, 240))
        ret, frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(frame.shape)
        
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.format = "jpeg"
        self.msg.data = frame.tobytes()
        self.publisher.publish(self.msg)
        cv2.waitKey(1)
def main() :
    rclpy.init()
    node = ImgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__' :
    main()