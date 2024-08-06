import socket
import struct
import pickle
import cv2
import torch
import requests
import threading
import time
from model import load_model

class Server:
    def __init__(self, host, port, odServer_url):
        self.host = host
        self.port = port
        self.odServer_url = odServer_url
        self.server_socket = None
        self.client_socket = None
        self.robot_model = None
        self.box_model = None
        self.person_model = None
        self.detected_boxes = []
        self.detected_robots = []
        self.detected_persons = []
        self.load_models()
        self.detect_thread = None
        self.send_thread = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.current_frame = None

    def load_models(self):
        try:
            self.robot_model = load_model('models/robot_best.pt')
            self.box_model = load_model('models/box_best.pt')
            self.person_model = load_model('models/yolov8n-pose.pt')
        except Exception as e:
            print(f"모델 로드 중 오류 발생: {e}")

    def run(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(10)
            print('클라이언트 연결 대기')
            self.client_socket, address = self.server_socket.accept()
            print('클라이언트 IP 주소:', address[0])
            self.detect_thread = threading.Thread(target=self.detect_objects_loop)
            self.send_thread = threading.Thread(target=self.send_detection_results_loop)
            self.detect_thread.start()
            self.send_thread.start()
            self.receive_frames()
        except Exception as e:
            print(f"예외 발생: {e}")
        finally:
            self.running = False
            if self.detect_thread:
                self.detect_thread.join()
            if self.send_thread:
                self.send_thread.join()
            self.close_connections()
            cv2.destroyAllWindows()
            print('연결 종료')

    def receive_frames(self):
        try:
            while self.running:
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
                print("수신 프레임 크기: {} bytes".format(data_size))
                try:
                    encoded_frame = pickle.loads(frame_data)
                    frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)
                    if frame is None:
                        print("프레임 디코딩 실패")
                        continue
                    print(f"프레임 사이즈: {frame.shape}")
                    cv2.imshow('Frame', frame)
                    cv2.waitKey(1)
                    with self.frame_lock:
                        self.current_frame = frame
                except Exception as e:
                    print(f"프레임 처리 중 오류 발생: {e}")
        except Exception as e:
            print(f"예외 발생: {e}")

    def detect_objects_loop(self):
        while self.running:
            with self.frame_lock:
                if self.current_frame is not None:
                    frame = self.current_frame
                else:
                    time.sleep(0.1)
                    continue
            input_tensor, (original_width, original_height) = self.preprocess_image(frame)
            with torch.no_grad():
                box_outputs = self.box_model(input_tensor) if self.box_model else None
                robot_outputs = self.robot_model(input_tensor) if self.robot_model else None
                person_outputs = self.person_model(input_tensor) if self.person_model else None
            self.detected_boxes = self.postprocess_outputs(box_outputs, original_width, original_height) if box_outputs else []
            self.detected_robots = self.postprocess_outputs(robot_outputs, original_width, original_height) if robot_outputs else []
            self.detected_persons = self.postprocess_outputs(person_outputs, original_width, original_height) if person_outputs else []
            print("box가 인식되었습니다:", self.detected_boxes)
            print("robot가 인식되었습니다:", self.detected_robots)
            print("person가 인식되었습니다:", self.detected_persons)
            time.sleep(1)

    def send_detection_results_loop(self):
        while self.running:
            results = {
                "boxes": 1 if self.detected_boxes else 0,
                "robots": 1 if self.detected_robots else 0,
                "persons": 1 if self.detected_persons else 0
            }
            try:
                response = requests.post(self.odServer_url, json=results)
                response.raise_for_status()
                print("객체 감지 결과 전송 성공:", response.status_code)
            except requests.exceptions.RequestException as e:
                print(f"객체 감지 결과 전송 중 오류 발생: {e}")
            time.sleep(1)

    def draw_boxes(self, frame, boxes, color, label):
        for (x_min, y_min, x_max, y_max, confidence) in boxes:
            cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
            label_with_confidence = f"{label} {confidence:.2f}"
            cv2.putText(frame, label_with_confidence, (int(x_min), int(y_min)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)


    def preprocess_image(self, frame):
        height, width = frame.shape[:2]
        resized_frame = cv2.resize(frame, (320, 320))
        resized_frame = resized_frame.transpose((2, 0, 1))
        input_tensor = torch.tensor(resized_frame).float() / 255.0
        input_tensor = input_tensor.unsqueeze(0)
        return input_tensor, (width, height)

    def postprocess_outputs(self, outputs, original_width, original_height):
        detected_boxes = []
        threshold = 0.75
        if outputs:
            results = outputs[0]
            boxes = results.boxes
            for box in boxes:
                confidence = box.conf[0].item()
                if confidence >= threshold:
                    x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
                    x_min = x_min * original_width / 320
                    y_min = y_min * original_height / 320
                    x_max = x_max * original_width / 320
                    y_max = y_max * original_height / 320
                    detected_boxes.append((x_min, y_min, x_max, y_max, confidence))
        return detected_boxes

    def close_connections(self):
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

if __name__ == "__main__":
    HOST = ''
    PORT = 
    ODSERVER_URL = ''
    server = Server(HOST, PORT, ODSERVER_URL)
    server.run()
