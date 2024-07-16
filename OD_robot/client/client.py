import cv2
import socket
import pickle
import struct
import time

class VideoClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.clientSocket = None
        self.capture = None
    
    def run(self):
        try:
            self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.clientSocket.connect((self.host, self.port))
            print("연결 성공")
    
            self.capture = cv2.VideoCapture(0)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            self.sendFrames()
        
        except Exception as e:
            print(f"예외 발생: {e}")
        
        finally:
            self.closeConnections()
            cv2.destroyAllWindows()
            print('연결 종료')

    
    def sendFrames(self):
        try:
            while True:
                retval, frame = self.capture.read()
                if not retval:
                    print("프레임 읽기 실패")
                    break

                retval, encodedFrame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not retval:
                    print("프레임 인코딩 실패")
                    continue
                
                frameData = pickle.dumps(encodedFrame)
                frameSize = len(frameData)
                
                self.clientSocket.sendall(struct.pack(">L", frameSize))
                self.clientSocket.sendall(frameData)
               
                time.sleep(0.2)  
        
        except Exception as e:
            print(f"예외 발생: {e}")
    
    def closeConnections(self):
        if self.clientSocket:
            self.clientSocket.close()
        
        if self.capture:
            self.capture.release()

if __name__ == "__main__":
    HOST = ''
    PORT = 8080
    videoClient = VideoClient(HOST, PORT)
    videoClient.run()
