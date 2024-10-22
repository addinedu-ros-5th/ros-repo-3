import sys
import os
import yaml
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QTransform, QPainter, QPen
from PyQt5.QtCore import Qt, QTimer
from threading import Thread

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped

# 현재 파일의 디렉토리 경로를 가져옵니다.
current_dir = os.path.dirname(os.path.abspath(__file__))

# .ui 파일의 상대 경로를 설정합니다.
ui_file = os.path.join(current_dir, 'odm.ui')

# uic 모듈을 사용하여 .ui 파일을 로드합니다.
from_class = uic.loadUiType(ui_file)[0]

# map.yaml 파일의 경로를 설정합니다.
yaml_file = os.path.join(current_dir, 'map.yaml')

# map.yaml 파일을 읽고 이미지 파일 경로를 가져옵니다.
with open(yaml_file, 'r') as file:
    map_yaml_data = yaml.full_load(file)
    image_file = map_yaml_data['image']

# 이미지 파일의 절대 경로를 설정합니다.
image_path = os.path.join(current_dir, image_file)

# 전역 변수 설정
robot_positions = {
    'robot1': [-2.75, -1.6],  # 로봇 1의 초기 위치(x, y)
    'robot2': [-2.85, -1.6],  # 로봇 2의 초기 위치(x, y)
    'robot3': [-2.95, -1.6],  # 로봇 3의 초기 위치(x, y)
    'robot4': [-3.05, -1.6]   # 로봇 4의 초기 위치(x, y)
}

# AmclSubscriber 클래스 정의
class AmclSubscriber(Node):
    def __init__(self, robot_name, topic_name):
        super().__init__(f'{robot_name}_amcl_subscriber')
        self.robot_name = robot_name
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            topic_name,
            self.amcl_callback,
            10
        )

    def amcl_callback(self, msg):
        global robot_positions
        robot_positions[self.robot_name][0] = msg.pose.pose.position.x
        robot_positions[self.robot_name][1] = msg.pose.pose.position.y

# 메인 윈도우 클래스 정의
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("OD WMS")
        self.load_map_image()

        # 타이머 설정
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(200)

    def load_map_image(self):
        # map QLabel 객체 가져오기
        self.map_label = self.findChild(QLabel, 'map')

        # 이미지 불러오기
        self.pixmap = QPixmap(image_path)

        # 이미지 크기 가져오기
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()

        # 이미지 스케일 설정
        self.image_scale = 12

        # 이미지 변환 (-90도 회전)
        transform = QTransform().rotate(-90)
        rotated_pixmap = self.pixmap.transformed(transform)

        # 이미지 이동 설정 (왼쪽으로 20픽셀 이동, 위로 20픽셀 이동)
        translated_pixmap = QPixmap(rotated_pixmap.size())

        painter = QPainter(translated_pixmap)
        move_x = 20  # 왼쪽으로 20픽셀 이동
        move_y = 8  # 위로 20픽셀 이동
        painter.drawPixmap(-move_x, -move_y, rotated_pixmap)  # x 좌표를 -로 설정하여 왼쪽으로 이동
        painter.end()

        # QLabel 크기에 맞게 이미지 조정 및 설정
        scaled_pixmap = translated_pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio)
        self.map_label.setPixmap(scaled_pixmap)

        # map resolution 및 origin 설정
        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]

    def update_map(self):
        # 기존 pixmap을 기반으로 QPixmap 생성
        updated_pixmap = QPixmap(self.map_label.pixmap())
        painter = QPainter(updated_pixmap)

        # 로봇 위치 업데이트
        global robot_positions

        # 각 로봇의 위치를 업데이트하고 표시합니다.
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
            painter.drawText(grid_x + 10, grid_y, robot_name[-1])  # 로봇 번호 표시

        painter.end()

        # QLabel에 업데이트된 pixmap 설정
        self.map_label.setPixmap(updated_pixmap)

    def calc_grid_position(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution * self.image_scale
        pos_y = (y - self.map_origin[1]) / self.map_resolution * self.image_scale
        return int((self.width - pos_x)), int(pos_y)

def main():
    rp.init()
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()

    # AmclSubscriber 노드 추가
    amcl_subscriber_1 = AmclSubscriber('robot1', 'amcl_pose')
    amcl_subscriber_2 = AmclSubscriber('robot2', 'amcl_pose_2')
    amcl_subscriber_3 = AmclSubscriber('robot3', 'amcl_pose_3')
    amcl_subscriber_4 = AmclSubscriber('robot4', 'amcl_pose_4')
    executor.add_node(amcl_subscriber_1)
    executor.add_node(amcl_subscriber_2)
    executor.add_node(amcl_subscriber_3)
    executor.add_node(amcl_subscriber_4)

    # ROS2 스레드 실행
    thread = Thread(target=executor.spin)
    thread.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
