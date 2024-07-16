import socket
import struct
import pickle
import cv2
import torch
import threading
from model import load_model

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.model = None
        self.detected_boxes = []

        self.load_model()

    def load_model(self):
        self.model = load_model('model/best.pt')
        
    def run(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(10)
            
            print('클라이언트 연결 대기')
            
            self.client_socket, address = self.server_socket.accept()
            print('클라이언트 ip 주소 :', address[0])
            
            self.receive_frames()
        
        except Exception as e:
            print(f"예외 발생: {e}")
        
        finally:
            self.close_connections()
            cv2.destroyAllWindows()
            print('연결 종료')
    
    def receive_frames(self):
        try:
            while True:
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
                
                print("수신 프레임 크기 : {} bytes".format(data_size))
                
                encoded_frame = pickle.loads(frame_data)
                frame = cv2.imdecode(encoded_frame, cv2.IMREAD_COLOR)
                
                detection_thread = threading.Thread(target=self.detect_objects, args=(frame,))
                detection_thread.start()
                
                detection_thread.join()

                self.draw_boxes(frame, self.detected_boxes)
                
                cv2.imshow('Frame', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        except Exception as e:
            print(f"예외 발생: {e}")

    def detect_objects(self, frame):
       
        input_tensor = self.preprocess_image(frame)

        with torch.no_grad():
            outputs = self.model(input_tensor)

        self.detected_boxes = self.postprocess_outputs(outputs)

        if self.detected_boxes:
            print("box가 인식되었습니다:", self.detected_boxes)
    
    def draw_boxes(self, frame, boxes):
        for (x_min, y_min, x_max, y_max) in boxes:
            cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 0, 255), 2)

    def preprocess_image(self, frame):
        resized_frame = cv2.resize(frame, (416, 416))
        resized_frame = resized_frame.transpose((2, 0, 1))
        input_tensor = torch.tensor(resized_frame).float() / 255.0
        input_tensor = input_tensor.unsqueeze(0)
        return input_tensor

    def postprocess_outputs(self, outputs):
        detected_boxes = []

        results = outputs[0] 
        boxes = results.boxes

        for box in boxes:
            x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
            detected_boxes.append((x_min, y_min, x_max, y_max))

        return detected_boxes

    def close_connections(self):
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

if __name__ == "__main__":
    host = ''
    port = 
    video_server = Server(host, port)
    video_server.run()
