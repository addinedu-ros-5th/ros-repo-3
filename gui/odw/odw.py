import os
import sys
import requests
from datetime import datetime

from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

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

    def ui_init(self):
        self.setupUi(self)
        self.setWindowTitle("OD Worker")
        self.stackedWidget.setCurrentWidget(self.order_page)
        self.orders_displayed = set()

        self.font = QFont()
        self.font.setPointSize(20)
        self.widgets = {}

        self.date_edit.setCalendarPopup(True)
        self.date_edit.setDisplayFormat("yyyy-MM-dd")
        self.date_edit.dateChanged.connect(self.update_data)

    def pulling(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1000)

    def check_new_data(self, selected_date):
        url = f"http://address:port/manager/order?date={selected_date}"
        response = requests.get(url)
        response.raise_for_status()
        data = response.json()
        print(data)
        return data
    
    def update_data(self):
        selected_date = self.date_edit.date().toString("yyyyMMdd")
        data = self.check_new_data(selected_date)
        self.display_orders(data, selected_date)

    def display_orders(self, data):
        items = data.get("order", [])
        current_orders = set()

        new_widgets = {}
        for item in items:
            order_id = item[0]
            order_time = item[1]
            date_format = "%a, %d %b %Y %H:%M:%S %Z"
            date_obj = datetime.strptime(order_time, date_format)
            order_time_str = date_obj.strftime("%Y-%m-%d")
            current_orders.add(order_id)

            if order_id in self.widgets:
                continue

            order_widget = QWidget()
            order_layout = QHBoxLayout(order_widget)
            order_id_label = QLabel(str(order_id))
            order_id_label.setFont(self.font)
            order_id_label.setAlignment(Qt.AlignCenter)
            order_time_label = QLabel(str(order_time_str))
            order_time_label.setAlignment(Qt.AlignCenter)
            order_button = QPushButton("확인")
            order_button.clicked.connect(lambda _, id=order_id: self.show_order_details(id))
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

    def show_order_details(self):
        # TODO
        pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OutboundDeliveryWorkerClass()
    window.show()
    sys.exit(app.exec_())
