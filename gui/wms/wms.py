import os
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5 import uic
from threading import Thread

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from wms_interfaces.msg import RobotStatusList

# 현재 파일의 디렉토리 경로를 가져옵니다.
current_dir = os.path.dirname(os.path.abspath(__file__))

# .ui 파일의 상대 경로를 설정합니다.
ui_file = os.path.join(current_dir, 'wms.ui')

# uic 모듈을 사용하여 .ui 파일을 로드합니다.
from_class = uic.loadUiType(ui_file)[0]

# ROS 2 노드 정의
class RobotStatusSubscriber(Node):
    def __init__(self, ui):
        super().__init__('robot_status_subscriber')
        self.ui = ui
        self.subscription = self.create_subscription(
            RobotStatusList,
            'robot_status',
            self.callback,
            10
        )

    def callback(self, msg):
        self.ui.task_state.setRowCount(4)
        self.ui.task_state.setItem(0, 0, QTableWidgetItem(msg.robot1.status))
        self.ui.task_state.setItem(0, 1, QTableWidgetItem(msg.robot1.task))
        self.ui.task_state.setItem(0, 2, QTableWidgetItem(msg.robot1.goal))
        
        self.ui.task_state.setItem(1, 0, QTableWidgetItem(msg.robot2.status))
        self.ui.task_state.setItem(1, 1, QTableWidgetItem(msg.robot2.task))
        self.ui.task_state.setItem(1, 2, QTableWidgetItem(msg.robot2.goal))
        
        self.ui.task_state.setItem(2, 0, QTableWidgetItem(msg.robot3.status))
        self.ui.task_state.setItem(2, 1, QTableWidgetItem(msg.robot3.task))
        self.ui.task_state.setItem(2, 2, QTableWidgetItem(msg.robot3.goal))
        
        self.ui.task_state.setItem(3, 0, QTableWidgetItem(msg.robot4.status))
        self.ui.task_state.setItem(3, 1, QTableWidgetItem(msg.robot4.task))
        self.ui.task_state.setItem(3, 2, QTableWidgetItem(msg.robot4.goal))

# 메인 윈도우 클래스 정의
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("OD WMS")
        self.init_table()

    def init_table(self):
        # 테이블 위젯 초기화
        self.task_state.setRowCount(4)
        self.task_state.setColumnCount(3)
        self.task_state.setHorizontalHeaderLabels(["Status", "Task", "Goal"])

def main():
    rp.init()
    executor = MultiThreadedExecutor()
    
    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()

    robot_status_subscriber = RobotStatusSubscriber(window)
    executor.add_node(robot_status_subscriber)

    thread = Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
