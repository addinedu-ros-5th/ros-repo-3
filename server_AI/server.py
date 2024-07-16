import socket
import struct
import pickle
import cv2
import torch
import threading
from model import loadModel

class VideoServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.serverSocket = None
        self.clientSocket = None
        self.model = None
        self.detectedBoxes = []

        self.loadModel()

    def loadModel(self):
        self.model = loadModel('model/best.pt')
        
    def run(self):
        try:
            self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.serverSocket.bind((self.host, self.port))
            self.serverSocket.listen(10)
            
            print('클라이언트 연결 대기')
            
            self.clientSocket, address = self.serverSocket.accept()
            print('클라이언트 ip 주소 :', address[0])
            
            self.receiveFrames()
        
        except Exception as e:
            print(f"예외 발생: {e}")
        
        finally:
            self.closeConnections()
            cv2.destroyAllWindows()
            print('연결 종료')
    
    def receiveFrames(self):
        try:
            while True:
                dataSizePacked = self.clientSocket.recv(4)
                if not dataSizePacked:
                    break
                
                dataSize = struct.unpack(">L", dataSizePacked)[0]
                
                frameData = b""
                while len(frameData) < dataSize:
                    packet = self.clientSocket.recv(dataSize - len(frameData))
                    if not packet:
                        break
                    frameData += packet
                
                if len(frameData) < dataSize:
                    print("연결 종료: 프레임 데이터를 모두 수신하지 못했습니다.")
                    break
                
                print("수신 프레임 크기 : {} bytes".format(dataSize))
                
                encodedFrame = pickle.loads(frameData)
                frame = cv2.imdecode(encodedFrame, cv2.IMREAD_COLOR)
                
                detectionThread = threading.Thread(target=self.detectObjects, args=(frame,))
                detectionThread.start()
                
                detectionThread.join()

                self.drawBoxes(frame, self.detectedBoxes)
                
                cv2.imshow('Frame', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        except Exception as e:
            print(f"예외 발생: {e}")

    def detectObjects(self, frame):
       
        inputTensor = self.preprocessImage(frame)

        with torch.no_grad():
            outputs = self.model(inputTensor)

        self.detectedBoxes = self.postprocessOutputs(outputs)

        if self.detectedBoxes:
            print("box가 인식되었습니다:", self.detectedBoxes)
    
    def drawBoxes(self, frame, boxes):
        for (xMin, yMin, xMax, yMax) in boxes:
            cv2.rectangle(frame, (int(xMin), int(yMin)), (int(xMax), int(yMax)), (0, 0, 255), 2)

    def preprocessImage(self, frame):
        resizedFrame = cv2.resize(frame, (416, 416))
        resizedFrame = resizedFrame.transpose((2, 0, 1))
        inputTensor = torch.tensor(resizedFrame).float() / 255.0
        inputTensor = inputTensor.unsqueeze(0)
        return inputTensor

    def postprocessOutputs(self, outputs):
        detectedBoxes = []

        results = outputs[0] 
        boxes = results.boxes

        for box in boxes:
            xMin, yMin, xMax, yMax = box.xyxy[0].tolist()
            detectedBoxes.append((xMin, yMin, xMax, yMax))

        return detectedBoxes

    def closeConnections(self):
        if self.clientSocket:
            self.clientSocket.close()
        if self.serverSocket:
            self.serverSocket.close()

if __name__ == "__main__":
    HOST = '192.168.2.21'
    PORT = 8080
    videoServer = VideoServer(HOST, PORT)
    videoServer.run()
