import sys
import os
import yaml
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QPixmap, QTransform, QPainter
from PyQt5.QtCore import Qt

# 현재 파일의 디렉토리 경로를 가져옵니다.
current_dir = os.path.dirname(os.path.abspath(__file__))

# .ui 파일의 상대 경로를 설정합니다.
ui_file = os.path.join(current_dir, 'wms.ui')

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

# 메인 윈도우 클래스 정의
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("OD WMS")
        self.load_map_image()

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

        # 초기 위치 설정
        self.now_x = 0
        self.now_y = 0

        # map resolution 및 origin 설정
        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]

def main():
    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()