import os
import sys
import requests
from datetime import datetime
import mysql.connector

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

        self.order_details = []
        self.current_detail_index = 0

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

        self.detail_page = QWidget()
        self.detail_layout = QVBoxLayout(self.detail_page)

        self.detail_form_layout = QFormLayout()
        self.detail_layout.addLayout(self.detail_form_layout)

        self.next_button = QPushButton("다음 물품")
        self.next_button.clicked.connect(self.show_next_detail)
        self.detail_layout.addWidget(self.next_button)

        self.stackedWidget.addWidget(self.detail_page)

    def pulling(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(2000)

    def check_new_data(self, selected_date):
        url = f""
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
            order_id = item['order_id']
            order_time = item['order_time']
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

    def show_order_details(self, order_id):
        url = ""
        params = {'order_id': order_id}
        response = requests.get(url, params=params)
        response.raise_for_status()
        self.order_details = response.json()
        self.current_detail_index = 0
        self.show_next_detail()

    def show_next_detail(self):
        if self.current_detail_index < len(self.order_details):
            detail = self.order_details[self.current_detail_index]
            product_name = detail['product_name']
            quantity = str(detail['quantity'])

            image_path = self.get_image_path_from_db(product_name)
            pixmap = QPixmap(image_path) if image_path and os.path.exists(image_path) else QPixmap()

            for i in reversed(range(self.detail_form_layout.count())):
                item = self.detail_form_layout.itemAt(i)
                if item:
                    widget = item.widget()
                    if widget:
                        widget.deleteLater()

            product_label = QLabel(f"물품: {product_name}")
            quantity_label = QLabel(f"수량: {quantity}")
            product_label.setFont(self.font)
            quantity_label.setFont(self.font)

            image_label = QLabel()
            image_label.setPixmap(pixmap)
            image_label.setScaledContents(True)
            image_label.setFixedSize(300, 300)

            self.detail_form_layout.addRow(product_label)
            self.detail_form_layout.addRow(quantity_label)
            self.detail_form_layout.addRow(image_label)

            if self.current_detail_index < len(self.order_details) - 1:
                self.next_button.setText("다음 물품")
            else:
                self.next_button.setText("담기 완료")


            self.stackedWidget.setCurrentWidget(self.detail_page)
            self.current_detail_index += 1
        else:
            self.stackedWidget.setCurrentWidget(self.order_page)

    def get_image_path_from_db(self, product_name):
        db_config = {
            'user' : '',
            'password' : '',
            'host' : '',
            'database' : ''
        }

        try:
            conn = mysql.connector.connect(**db_config)
            cursor = conn.cursor(dictionary=True)

            query = "SELECT image_path FROM product_image_path WHERE product_name = %s"
            cursor.execute(query, (product_name,))
            result = cursor.fetchone()

            return result['image_path'] if result else None
        except mysql.connector.Error as err:
            print(f"Error: {err}")
            return None
        finally:
            if conn.is_connected():
                cursor.close()
                conn.close()

    def show_order_page(self):
        self.stackedWidget.setCurrentWidget(self.order_page)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OutboundDeliveryWorkerClass()
    window.show()
    sys.exit(app.exec_())
