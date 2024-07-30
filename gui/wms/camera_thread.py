import socket
import cv2
import struct
import pickle

from PyQt5.QtCore import QThread, pyqtSignal

class CameraThread(QThread):
    frame_received = pyqtSignal(object)

    def __init__(self, ip, port):
        super().__init__()
        self.ip = ip
        self.port = port
        self.client_socket = None
        self.running = True

    def run(self):
        self.connect_to_server()
        self.receive_frames()

    def connect_to_server(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.ip, self.port))
        except Exception as e:
            print(f"서버 연결 중 오류 발생: {e}")
            self.running = False

    def receive_frames(self):
        while self.running:
            try:
                data_size_packed = self.client_socket.recv(4)
                if not data_size_packed:
                    break

                data_size = struct.unpack(">L", data_size_packed)[0]
                frame_data = b""
                while len(frame_data) < data_size:
                    packet = self.client_socket.recv(data_size - len(frame_data))
                    if not packet:
                        break
                    frame_data += packet

                if len(frame_data) < data_size:
                    print("연결 종료: 프레임 데이터를 모두 수신하지 못했습니다.")
                    break

                encoded_frame = pickle.loads(frame_data)
                frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)
                self.frame_received.emit(frame)

            except Exception as e:
                print(f"프레임 수신 중 예외 발생: {e}")
                self.running = False
    
    def stop(self):
        self.running = False
        if self.client_socket:
            self.client_socket.close()
