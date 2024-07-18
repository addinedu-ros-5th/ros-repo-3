from ultralytics import YOLO

def loadModel(model_path):
    model = YOLO(model_path)
    return model
