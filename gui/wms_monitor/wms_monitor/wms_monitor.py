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
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

# 상수 설정
GOAL_REACHED_TOL_DEFAULT = 0.25

# 현재 파일의 디렉토리 경로를 가져옵니다.
current_dir = os.path.dirname(os.path.abspath(__file__))

# .ui 파일의 상대 경로를 설정합니다.
ui_file = os.path.join(current_dir, 'wms.ui')

# uic 모듈을 사용하여 .ui 파일을 로드합니다.
from_class = uic.loadUiType(ui_file)[0]

# map.yaml 파일의 상대 경로를 설정합니다.
yaml_file = os.path.join(current_dir, '..', 'map.yaml')

# map.yaml 파일을 읽고 이미지 파일 경로를 가져옵니다.
with open(yaml_file, 'r') as file:
    map_yaml_data = yaml.full_load(file)
    image_file = map_yaml_data['image']

# 이미지 파일의 상대 경로를 설정합니다.
image_path = os.path.join(current_dir, '..', image_file)

# 맵의 해상도와 원점 좌표 설정
map_resolution = map_yaml_data['resolution']
map_origin = map_yaml_data['origin'][:2]

# 전역 변수 설정
global amcl_1, amcl_2, amcl_3
amcl_1 = PoseWithCovarianceStamped()
amcl_2 = PoseWithCovarianceStamped()
amcl_3 = PoseWithCovarianceStamped()

# AmclSubscriber 클래스 정의
class AmclSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_subscriber')

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber1 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose_1',
            self.amcl_callback_1,
            amcl_pose_qos
        )

        self.subscriber2 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose_2',
            self.amcl_callback_2,
            amcl_pose_qos
        )

        self.subscriber3 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose_3',
            self.amcl_callback_3,
            amcl_pose_qos
        )

    def amcl_callback_1(self, msg):
        global amcl_1
        amcl_1 = msg
        #print(f"Updated robot1 position to {amcl_1.pose.pose.position}")  # 로그 출력 추가

    def amcl_callback_2(self, msg):
        global amcl_2
        amcl_2 = msg
        #print(f"Updated robot2 position to {amcl_2.pose.pose.position}")  # 로그 출력 추가

    def amcl_callback_3(self, msg):
        global amcl_3
        amcl_3 = msg
        #print(f"Updated robot3 position to {amcl_3.pose.pose.position}")  # 로그 출력 추가


# 메인 윈도우 클래스 정의
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("OD WMS")
        self.load_map_image()
        self.init_robot_location()

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

    def init_robot_location(self):
        # 기존 pixmap을 기반으로 QPixmap 생성
        updated_pixmap = QPixmap(self.map_label.pixmap())
        painter = QPainter(updated_pixmap)

        # 로봇 위치 업데이트
        robot_positions = {
            'robot1': [-2.80, -1.6],  # 로봇 1의 초기 위치(x, y)
            'robot2': [-2.95, -1.6],  # 로봇 2의 초기 위치(x, y)
            'robot3': [-3.10, -1.6]   # 로봇 3의 초기 위치(x, y)
        }

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
        print(f"Map updated with positions: {robot_positions}")  # 로그 출력 추가

    def update_map(self):
        # 기존 pixmap을 기반으로 QPixmap 생성
        updated_pixmap = QPixmap(self.map_label.pixmap())
        painter = QPainter(updated_pixmap)

        # 로봇 위치 업데이트
        global amcl_1, amcl_2, amcl_3

        # 각 로봇의 위치를 업데이트하고 표시합니다.
        if amcl_1.pose.pose.position:
            self.update_robot_position(painter, amcl_1.pose.pose.position, Qt.red, '1')
        if amcl_2.pose.pose.position:
            self.update_robot_position(painter, amcl_2.pose.pose.position, Qt.green, '2')
        if amcl_3.pose.pose.position:
            self.update_robot_position(painter, amcl_3.pose.pose.position, Qt.blue, '3')

        painter.end()

        # QLabel에 업데이트된 pixmap 설정
        self.map_label.setPixmap(updated_pixmap)
        #print(f"Map updated with positions: {amcl_1.pose.pose.position}, {amcl_2.pose.pose.position}, {amcl_3.pose.pose.position}")

    def update_robot_position(self, painter, position, color, label):
        grid_x, grid_y = self.calc_grid_position(position.x, position.y)
        pen = QPen(color, 10)
        painter.setPen(pen)
        painter.drawPoint(grid_x, grid_y)
        painter.drawText(grid_x + 10, grid_y, label)  # 로봇 번호 표시

    def calc_grid_position(self, x, y):
        # 이미지 스케일에 따른 좌표 변환
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
        return int((self.width - pos_x * self.image_scale)), int(pos_y * self.image_scale)

    def closeEvent(self, event):
        rp.shutdown()  # ROS2 종료
        super().closeEvent(event)  # 윈도우 종료

def main():
    rp.init()
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()

    # AmclSubscriber 노드 추가
    amcl_subscriber = AmclSubscriber()
    executor.add_node(amcl_subscriber)

    # ROS2 스레드 실행
    thread = Thread(target=executor.spin)
    thread.start()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
