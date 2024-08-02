import cv2
import socket
import pickle
import struct
import time
import threading

class Client:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.clientSocket = None
        self.capture = None
        self.running = True
        self.send_thread = None 

    def run(self):
        try:
            self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.clientSocket.connect((self.host, self.port))
            print("연결 성공")
    
            self.capture = cv2.VideoCapture(0)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
            
            self.send_thread = threading.Thread(target=self.sendFrames)
            self.send_thread.start()
        
            while self.running:
                time.sleep(1) 
        
        except Exception as e:
            print(f"예외 발생: {e}")
        
        finally:
            self.running = False 
            if self.send_thread:
                self.send_thread.join()
            self.closeConnections()
            cv2.destroyAllWindows()
            print('연결 종료')

    def sendFrames(self):
        frame_count = 0 
        
        try:
            while self.running:
                retval, frame = self.capture.read()
                if not retval:
                    print("프레임 읽기 실패")
                    break

                frame_count += 1
                
                if frame_count % 10 == 0:
                    retval, encodedFrame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if not retval:
                        print("프레임 인코딩 실패")
                        continue
                    
                    frameData = pickle.dumps(encodedFrame)
                    frameSize = len(frameData)
          
                    self.clientSocket.sendall(struct.pack(">L", frameSize))
                    self.clientSocket.sendall(frameData)
                    print("프레임 전송 완료")
                
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
    PORT = 
    videoClient = Client(HOST, PORT)
    videoClient.run()
