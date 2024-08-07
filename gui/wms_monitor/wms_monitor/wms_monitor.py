import os
import sys
import yaml
import math
import json
import requests
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from threading import Thread

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from outbound_delivery_robot_interfaces.msg import AStar

current_dir = os.path.dirname(os.path.abspath(__file__))

ui_file = os.path.join(current_dir, 'wms.ui')
from_class = uic.loadUiType(ui_file)[0]

yaml_file = os.path.join(current_dir, 'map.yaml')

print(yaml_file)

with open(yaml_file, 'r') as file:
    map_yaml_data = yaml.full_load(file)
    image_file = map_yaml_data['image']

image_path = os.path.join(current_dir, image_file)

map_resolution = map_yaml_data['resolution']
map_origin = map_yaml_data['origin'][:2]

global amcl_1, amcl_2, amcl_3
amcl_1 = PoseWithCovarianceStamped()
amcl_2 = PoseWithCovarianceStamped()
amcl_3 = PoseWithCovarianceStamped()

global path_1, path_2, path_3, path_before_1, path_before_2, path_before_3
path_1 = AStar()
path_2 = AStar()
path_3 = AStar()
path_before_1 = AStar()
path_before_2 = AStar()
path_before_3 = AStar()

global start_point_1, start_point_2, start_point_3
start_point_1, start_point_2, start_point_3 = None, None, None

# QoS 설정
amcl_pose_qos = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# AmclSubscriber 클래스 정의
class AmclSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_subscriber')

        self.pose1 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback1,
            amcl_pose_qos)
        
        self.pose2 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose_1',
            self.amcl_callback2,
            amcl_pose_qos)
        
        self.pose3 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose_2',
            self.amcl_callback3,
            amcl_pose_qos)

    def amcl_callback1(self, amcl):
        global amcl_1
        amcl_1 = amcl
        
    def amcl_callback2(self, amcl):
        global amcl_2
        amcl_2 = amcl
        
    def amcl_callback3(self, amcl):
        global amcl_3
        amcl_3 = amcl
        
class PathSubscriber(Node):
    
    def __init__(self):
        super().__init__('path_subscriber')
        
        self.sub1 = self.create_subscription(
            AStar,
            'planned_path_1',
            self.path_callback1,
            10
        )
    
        self.sub2 = self.create_subscription(
            AStar,
            'planned_path_2',
            self.path_callback2,
            10
        )
        
        self.sub3 = self.create_subscription(
            AStar,
            'planned_path_3',
            self.path_callback3,
            10
        )
        
    def path_callback1(self, path):
        global path_1, amcl_1, start_point_1
        path_1 = path
        start_point_1 = amcl_1
        
    def path_callback2(self, path):
        global path_2, amcl_2, start_point_2
        path_2 = path
        start_point_2 = amcl_2
        
    def path_callback3(self, path):
        global path_3, amcl_3, start_point_3
        path_3 = path
        start_point_3 = amcl_3

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("OD WMS")
        self.map_label = self.findChild(QLabel, 'map')
        self.robot_table = self.findChild(QTableWidget, 'task_state')

        if self.robot_table is None:
            print("Failed to find task_state. Check the UI file for the correct widget name.")
        else:
            print("task_state found successfully.")
            self.robot_table.setColumnCount(5)
            self.robot_table.setHorizontalHeaderLabels(["Date", "ID", "Task", "Goal", "Status"])

            # 로봇 ID 설정
            for row in range(3):
                self.robot_table.setItem(row, 1, QTableWidgetItem(f"od_robot{row+1}"))

        self.load_map_image()

        # 타이머 설정
        self.map_timer = QTimer(self)
        self.map_timer.timeout.connect(self.update_map)
        self.map_timer.start(200)
        
        # 서버 전송 주기 타이머 설정
        self.server_timer = QTimer(self)
        self.server_timer.timeout.connect(self.update_table)
        self.server_timer.start(2000)

    def load_map_image(self):
        self.pixmap = QPixmap(image_path)
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.image_scale = 12.3

        # 이미지를 -90도 회전하고 이동합니다.
        transform = QTransform().rotate(-90)
        rotated_pixmap = self.pixmap.transformed(transform)

        translated_pixmap = QPixmap(rotated_pixmap.size())
        painter = QPainter(translated_pixmap)
        move_x = -21.5
        move_y = -8
        painter.drawPixmap(int(move_x), int(move_y), rotated_pixmap)
        painter.end()

        # QLabel 크기에 맞게 이미지 조정 및 설정
        scaled_pixmap = translated_pixmap.scaled(int(self.width * self.image_scale), int(self.height * self.image_scale), Qt.KeepAspectRatio)
        self.map_label.setPixmap(scaled_pixmap)

        # map resolution 및 origin 설정
        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]

        # 초기 pixmap 저장
        self.initial_pixmap = scaled_pixmap

    def update_map(self):
        updated_pixmap = QPixmap(self.initial_pixmap)
        painter = QPainter(updated_pixmap)
        
        # 로봇 번호 표시
        self.font = QFont()
        self.font.setBold(True)
        self.font.setPointSize(15)
        painter.setFont(self.font)
        
        try:
            # 1번 로봇 좌표 및 방향
            x1, y1 = self.calc_grid_position(amcl_1.pose.pose.position.y, amcl_1.pose.pose.position.x)
            x1 = self.width - x1
            theta1 = self.get_yaw(amcl_1.pose.pose.orientation)
            painter.setPen(QPen(Qt.red, 10, Qt.SolidLine))
            painter.drawPoint(int((x1 - 50) * self.image_scale), int((self.height - y1 + 2) * self.image_scale))
            painter.drawText(int((x1 - 48) * self.image_scale), int((self.height - y1 + 2) * self.image_scale + 5), '1')
            
            # 2번 로봇 좌표 및 방향
            x2, y2 = self.calc_grid_position(amcl_2.pose.pose.position.y, amcl_2.pose.pose.position.x)
            x2 = self.width - x2
            theta2 = self.get_yaw(amcl_2.pose.pose.orientation)
            painter.setPen(QPen(Qt.blue, 10, Qt.SolidLine))
            painter.drawPoint(int((x2 - 50) * self.image_scale), int((self.height - y2 + 2) * self.image_scale))
            painter.drawText(int((x2 - 48) * self.image_scale), int((self.height - y2 + 2) * self.image_scale + 5), '2')

            # 3번 로봇 좌표 및 방향
            x3, y3 = self.calc_grid_position(amcl_3.pose.pose.position.y, amcl_3.pose.pose.position.x)
            theta3 = self.get_yaw(amcl_3.pose.pose.orientation)
            painter.setPen(QPen(Qt.green, 10, Qt.SolidLine))
            painter.drawPoint(int((x3 - 50) * self.image_scale), int((self.height - y3 + 2) * self.image_scale))
            painter.drawText(int((x3 - 48) * self.image_scale), int((self.height - y3 + 2) * self.image_scale + 5), '3')
        
        finally:
            painter.end()

        self.map_label.setPixmap(updated_pixmap)
    
    def calc_grid_position(self, x, y):
        grid_x = (x - self.map_origin[0]) / self.map_resolution 
        grid_y = (y - self.map_origin[1]) / self.map_resolution  
        return int(grid_x), int(grid_y)

    def get_yaw(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y) 
        cosy_cosp = 1 - 2 * (orientation.y * (orientation.y + orientation.z) ) 
        return math.atan2(siny_cosp, cosy_cosp)

    def fetch_data(self):
        url = "http://192.168.1.100:5000/gui/info"
        json_data = {"request": "call"}
        
        try:
            response = requests.post(url, json=json_data)
            response.raise_for_status()  # HTTP 오류 발생 시 예외 발생
            data = response.json()
            print(data)
            return data
        except requests.exceptions.RequestException as e:
            print(f"Error fetching data: {e}")
            return None

    def update_table(self):
        items = self.fetch_data()
        if items:
            self.robot_table.setRowCount(len(items))
            for i, item in enumerate(items):
                self.robot_table.setItem(i, 0, QTableWidgetItem(str(item['order_date'])))
                self.robot_table.setItem(i, 1, QTableWidgetItem(str(item['order_id'])))
                self.robot_table.setItem(i, 2, QTableWidgetItem(str(item['robot_task'])))
                self.robot_table.setItem(i, 3, QTableWidgetItem(str(item['robot_goal'])))
                self.robot_table.setItem(i, 4, QTableWidgetItem(str(item['robot_status'])))
        else:
            print("No data to display in the table.")

def ros_thread():
    rp.init()
    amcl_node = AmclSubscriber()
    path_node = PathSubscriber()
    
    executor = MultiThreadedExecutor()
    executor.add_node(amcl_node)
    executor.add_node(path_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        amcl_node.destroy_node()
        path_node.destroy_node()
        rp.shutdown()

if __name__ == "__main__":
    ros_thread_instance = Thread(target=ros_thread)
    ros_thread_instance.start()
    
    app = QApplication(sys.argv)
    mainWindow = WindowClass()
    mainWindow.show()
    sys.exit(app.exec_())
