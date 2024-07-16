import socket
import struct
import pickle
import cv2
import torch
from model import load_model

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.model = None

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
                self.detect_objects(frame)
                cv2.imshow('Frame', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        except Exception as e:
            print(f"예외 발생: {e}")

    def preprocess_image(self, frame):
        resized_frame = cv2.resize(frame, (416, 416))
        resized_frame = resized_frame.transpose((2, 0, 1))  
        input_tensor = torch.tensor(resized_frame).float() / 255.0  
        input_tensor = input_tensor.unsqueeze(0)  
        return input_tensor

    def detect_objects(self, frame):
        input_tensor = self.preprocess_image(frame)

        with torch.no_grad():
            outputs = self.model(input_tensor)
        
        detected_boxes = self.postprocess_outputs(outputs)
        
        for box in detected_boxes:
            print("box가 인식되었습니다:", box)
    
    def postprocess_outputs(self, outputs):
        detected_boxes = []

        for output in outputs[0]: 
            if len(output) >= 4:
                x_center, y_center, width, height = output[:4]
                x_min = x_center - width / 2
                y_min = y_center - height / 2
                x_max = x_center + width / 2
                y_max = y_center + height / 2
                detected_boxes.append((x_min, y_min, x_max, y_max))
        
        return detected_boxes

    def close_connections(self):
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

if __name__ == "__main__":
    host = '192.168.2.21'
    port = 8080
    video_server = Server(host, port)
    video_server.run()
