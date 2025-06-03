import torch
import warnings

# i've globally patched torch.load to use weights_only=False, something with the version of ultralytics
original_load = torch.load
def patched_load(*args, **kwargs):
    kwargs['weights_only'] = False
    return original_load(*args, **kwargs)
torch.load = patched_load

warnings.filterwarnings("ignore")

import cv2
from ultralytics import YOLO

print("Loading YOLOv8 model ...")
model = YOLO('yolov8n.pt')
print("Model loaded successfully")


cap = cv2.VideoCapture("http://192.168.0.112:8080/video")

if not cap.isOpened():
    print("Cannot connect to camera")
    exit()

print("Camera connected! Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    results = model(frame)
    
    annotated_frame = results[0].plot()
    
    cv2.imshow('Object Detection', annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()