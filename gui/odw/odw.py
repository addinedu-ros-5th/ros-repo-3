import os
import sys
import requests
import psutil
import socket

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from functools import partial


def path():
    current_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_path)
    return current_dir

current_dir = path()
form = uic.loadUiType(current_dir + "/odw.ui")[0]

class OutboundDeliveryWorkerClass(QMainWindow, form):
    def __init__(self):
        super().__init__()

        self.ui_init()
        self.pulling()
        self.update_data()
        
    def get_local_ip(self):
        interfaces = psutil.net_if_addrs()
        for interface, addrs in interfaces.items():
            if 'wlo1' in interface.lower():
                for addr in addrs:
                    if addr.family == socket.AF_INET:
                        return addr.address
        return None

    def ui_init(self):
        self.setupUi(self)
        self.setWindowTitle("OD Worker")
        self.stackedWidget.setCurrentWidget(self.order_page)
        self.orders_displayed = set()
        self.order_details_displayed = set()

        self.font = QFont()
        self.font.setPointSize(20)
        self.widgets = {}
        self.detail_widgets = {}

        self.date_edit.setCalendarPopup(True)
        self.date_edit.setDisplayFormat("yyyy-MM-dd")
        self.date_edit.setDate(QDate.currentDate())
        self.date_edit.dateChanged.connect(self.update_data)
        
        self.back_btn.clicked.connect(self.back)

        
    def back(self):
        self.stackedWidget.setCurrentWidget(self.order_page)
        
    def end(self):
        pass

    def pulling(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(2000)

    def check_new_data(self, selected_date):
        address = self.get_local_ip()
        url = f"http://{address}:5000/database/order?date={selected_date}"
        response = requests.get(url)
        response.raise_for_status()
        data = response.json()
        return data
    
    def update_data(self):
        selected_date = self.date_edit.date().toString("yyyyMMdd")
        data = self.check_new_data(selected_date)
        self.display_orders(data)

    def display_orders(self, data):
        current_orders = set()
        new_widgets = {}
        
        for item in data:
            order_id = item["order_id"]
            order_time = item["order_time"]
            current_orders.add(order_id)

            if order_id in self.widgets:
                continue

            order_widget = QWidget()
            order_layout = QHBoxLayout(order_widget)

            order_id_label = QLabel(str(order_id))
            order_id_label.setFont(self.font)
            order_id_label.setAlignment(Qt.AlignCenter)

            order_time_label = QLabel(order_time)
            order_time_label.setAlignment(Qt.AlignCenter)

            order_button = QPushButton("확인")
            order_button.clicked.connect(partial(self.get_order_details, order_id))

            order_layout.addWidget(order_id_label)
            order_layout.addWidget(order_time_label)
            order_layout.addWidget(order_button)

            self.order_list_layout.addWidget(order_widget)
            new_widgets[order_id] = order_widget

        orders_to_remove = self.orders_displayed - current_orders
        for order_id in orders_to_remove:
            widget = self.widgets.pop(order_id, None)
            if widget:
                self.order_list_layout.removeWidget(widget)
                widget.deleteLater()

        self.orders_displayed = current_orders
        self.widgets.update(new_widgets)

    def get_order_details(self, order_id):
        address = self.get_local_ip()
        url = f"http://{address}:5000/database/order_detail?order_id={order_id}"
        response = requests.get(url)
        response.raise_for_status()
        order_details = response.json()
        self.stackedWidget.setCurrentWidget(self.order_info)
        self.show_order_details(order_details, order_id)

    def show_order_details(self, order_details, order_id):
        
        self.assign_btn.clicked.connect(lambda: self.assign_robot(order_id))
        self.end_btn.clicked.connect(self.end)
        for i in reversed(range(self.order_detail_layout.count())):
            widget = self.order_detail_layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()

        self.detail_widgets.clear()

        new_widgets = {}
        for order_detail in order_details:
            product_name = order_detail["product_name"]
            quantity = str(order_detail["quantity"])

            image_path = path() + "/" + self.get_image_path(product_name)
            pixmap = QPixmap(image_path) if image_path and os.path.exists(image_path) else QPixmap()

            order_detail_widget = QWidget()
            order_detail_layout = QHBoxLayout(order_detail_widget)

            product_name_label = QLabel(product_name)
            product_name_label.setFont(self.font)
            product_name_label.setAlignment(Qt.AlignCenter)

            quantity_label = QLabel(quantity)
            quantity_label.setAlignment(Qt.AlignCenter)

            image_label = QLabel()
            image_label.setPixmap(pixmap)
            image_label.setScaledContents(True)
            image_label.setFixedSize(100, 100)

            order_detail_layout.addWidget(image_label)
            order_detail_layout.addWidget(product_name_label)
            order_detail_layout.addWidget(quantity_label)

            self.order_detail_layout.addWidget(order_detail_widget)
            new_widgets[order_id] = order_detail_widget

        self.detail_widgets = new_widgets

    def get_image_path(self, product_name):
        address = self.get_local_ip()
        url = f"http://{address}:5000/database/image"
        json = {'product': product_name}
        response = requests.post(url, json=json)
        response.raise_for_status()
        self.order_details = response.json()["image_path"]
        return self.order_details
        
    def assign_robot(self, order_id):
        address = self.get_local_ip()
        selected_date = self.date_edit.date().toString("yyyyMMdd")
        url = f"http://{address}:5000/task/manage"
        json = {'order_id': order_id, 'date': selected_date}
        response = requests.post(url, json=json)
        response.raise_for_status()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OutboundDeliveryWorkerClass()
    window.show()
    sys.exit(app.exec_())
