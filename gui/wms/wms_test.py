import sys
import os
import yaml
import cv2
from PyQt5 import uic
<<<<<<< HEAD
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QTransform, QPainter, QPen
from PyQt5.QtCore import Qt, QTimer
from threading import Thread
=======
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from camera_thread import CameraThread

>>>>>>> 70772c89aef41403186fbc76e20f57101f4ee531

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

current_dir = os.path.dirname(os.path.abspath(__file__))

ui_file = os.path.join(current_dir, 'wms.ui')

from_class = uic.loadUiType(ui_file)[0]

yaml_file = os.path.join(current_dir, 'map.yaml')

with open(yaml_file, 'r') as file:
    map_yaml_data = yaml.full_load(file)
    image_file = map_yaml_data['image']

image_path = os.path.join(current_dir, image_file)

map_resolution = map_yaml_data['resolution']
map_origin = map_yaml_data['origin'][:2]

robot_positions = {
    'robot1': [-2.75, -1.6], 
    'robot2': [-2.85, -1.6], 
    'robot3': [-2.95, -1.6],  
    'robot4': [-3.05, -1.6]   
}

class AmclSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_subscriber')

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscribers = {
            'robot1': self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose_1',
                self.amcl_callback_factory('robot1'),
                amcl_pose_qos
            ),
            'robot2': self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose_2',
                self.amcl_callback_factory('robot2'),
                amcl_pose_qos
            ),
            'robot3': self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose_3',
                self.amcl_callback_factory('robot3'),
                amcl_pose_qos
            ),
            'robot4': self.create_subscription(
                PoseWithCovarianceStamped,
<<<<<<< HEAD
                '/amcl_pose_4',
=======
                '/amcl_pose_4', 
>>>>>>> 70772c89aef41403186fbc76e20f57101f4ee531
                self.amcl_callback_factory('robot4'),
                amcl_pose_qos
            )
        }

    def amcl_callback_factory(self, robot_name):
        def amcl_callback(msg):
            global robot_positions
            robot_positions[robot_name][0] = msg.pose.pose.position.x
            robot_positions[robot_name][1] = msg.pose.pose.position.y
        return amcl_callback

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("OD WMS")
        self.load_map_image()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(200)

        self.cam_btn1.clicked.connect(self.btn_cam1)
        self.cam_btn2.clicked.connect(self.btn_cam2)
        self.cam_btn3.clicked.connect(self.btn_cam3)
        self.cam_btn4.clicked.connect(self.btn_cam4)

        self.camera_thread = None

    def load_map_image(self):
        self.map_label = self.findChild(QLabel, 'map')

        self.pixmap = QPixmap(image_path)

        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()

        self.image_scale = 12

        transform = QTransform().rotate(-90)
        rotated_pixmap = self.pixmap.transformed(transform)

        translated_pixmap = QPixmap(rotated_pixmap.size())

        painter = QPainter(translated_pixmap)
        move_x = 20  
        move_y = 8  
        painter.drawPixmap(-move_x, -move_y, rotated_pixmap)
        painter.end()

        scaled_pixmap = translated_pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio)
        self.map_label.setPixmap(scaled_pixmap)

        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]

    def update_map(self):
        updated_pixmap = QPixmap(self.map_label.pixmap())
        painter = QPainter(updated_pixmap)

        global robot_positions

        for robot_name, position in robot_positions.items():
            grid_x, grid_y = self.calc_grid_position(position[0], position[1])
            
            if robot_name == 'robot1':
                pen = QPen(Qt.red, 10)
            elif robot_name == 'robot2':
                pen = QPen(Qt.green, 10)
            elif robot_name == 'robot3':
                pen = QPen(Qt.blue, 10)
            elif robot_name == 'robot4':
                pen = QPen(Qt.yellow, 10)
            painter.setPen(pen)
            painter.drawPoint(grid_x, grid_y)
            painter.drawText(grid_x + 10, grid_y, robot_name[-1])  

        painter.end()

        self.map_label.setPixmap(updated_pixmap)

    def calc_grid_position(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution * self.image_scale
        pos_y = (y - self.map_origin[1]) / self.map_resolution * self.image_scale
        return int((self.width - pos_x)), int(pos_y)

<<<<<<< HEAD
=======
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            x = event.position().x()
            y = event.position().y()
            self.set_robot_position(x, y)

    def set_robot_position(self, x, y):
        global robot_positions

        map_x = (self.width - x) / self.image_scale * self.map_resolution + self.map_origin[0]
        map_y = y / self.image_scale * self.map_resolution + self.map_origin[1]

        robot_name = 'robot1'  

        robot_positions[robot_name] = [map_x, map_y]
        self.update_map()

    def update_cam_label(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        qimg = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        self.cam_label.setPixmap(pixmap)

    def btn_cam1(self):
        pass

    def btn_cam2(self):
        pass

    def btn_cam3(self):
        if not self.camera_thread:
            self.camera_thread = CameraThread('192.168.0.69', 8080)
            self.camera_thread.frame_received.connect(self.update_cam_label)
            self.camera_thread.start()

    def btn_cam4(self):
        pass

    def camera_event(self, event):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.wait()
        event.accept

>>>>>>> 70772c89aef41403186fbc76e20f57101f4ee531
    def closeEvent(self, event):
        rp.shutdown()
        super().closeEvent(event) 

def main():
    rp.init()
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()

    amcl_subscriber = AmclSubscriber()
    executor.add_node(amcl_subscriber)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
