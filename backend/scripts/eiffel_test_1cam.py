import time
import threading
import queue
from pathlib import Path

import cv2
from ultralytics import YOLO

import torch
from torchvision import transforms, models
from torchvision.models import ResNet18_Weights
from PIL import Image

from picamera2 import Picamera2

# --- Configuration Paths & Settings ---
MODEL_DETECT_PATH = 'models/bestbest_ncnn_model'
MODEL_CLASSIFY_PATHS = ['models/angle_classifier.pth']
OUTPUT_DIR = Path('dataset/webcam_crops')
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Confidence thresholds
Yolo_conf_thresh = 0.8
Classify_conf_thresh = 50.0  # percent

# Angle classes
ANGLE_CLASSES = ['0', '15', '30', '45', '60', '75', '105', '120', '150', '165']

# --- Model Loading & Prediction ---
_model_cache = {}

def load_classify_model(path):
    if path in _model_cache:
        return _model_cache[path]
    # load ResNet18 base
    m = models.resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
    m.fc = torch.nn.Linear(m.fc.in_features, len(ANGLE_CLASSES))
    checkpoint = torch.load(path, map_location='cpu')
    state = checkpoint.get('model_state_dict', checkpoint)
    m.load_state_dict(state)
    m.eval()
    _model_cache[path] = m
    return m

def predict_angle(frame, model):
    tf = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
    ])
    image = Image.fromarray(frame) if isinstance(frame, (bytes, bytearray)) or hasattr(frame, 'shape') else frame
    if image.mode != 'RGB': image = image.convert('RGB')
    inp = tf(image).unsqueeze(0)
    with torch.no_grad():
        out = model(inp)
        probs = torch.nn.functional.softmax(out, dim=1)[0]
        cls_idx = int(probs.argmax())
        conf = float(probs[cls_idx] * 100)
    return ANGLE_CLASSES[cls_idx], conf

# --- YOLO Detector ---
detector = YOLO(MODEL_DETECT_PATH, task='detect')
detector.conf = Yolo_conf_thresh

# --- Camera Thread Class ---
class CameraThread:
    def __init__(self, cam_id):
        self.cam_id = cam_id
        self.picam2 = Picamera2(camera_num=cam_id)
        self.queue = queue.Queue(maxsize=2)
        # preview config
        cfg = self.picam2.create_preview_configuration(
            main={'size': (1280,960), 'format': 'RGB888'},
            lores={'size': (640,480), 'format': 'YUV420'}
        )
        self.picam2.configure(cfg)
        self.running = False

    def start(self):
        self.running = True
        self.picam2.start()
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.running = False
        self.picam2.stop()

    def _run(self):
        while self.running:
            frame = self.picam2.capture_array()
            if not self.queue.full():
                self.queue.put(frame)

    def get_frame(self):
        return self.queue.get_nowait() if not self.queue.empty() else None

# --- Frame Processing ---
def process_frame(frame, cam_id, models_classify):
    # rotate and detect
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    res = detector(frame)[0]
    boxes = res.boxes
    best = max(boxes, key=lambda b: b.conf[0]) if boxes else None
    crop = None
    angle_label = None
    if best:
        x1,y1,x2,y2 = map(int, best.xyxy[0].tolist())
        # draw bounding box
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        # crop
        crop = frame[y1:y2, x1:x2]
        # classify
        m = load_classify_model(models_classify[0])
        label, conf = predict_angle(crop, m)
        angle_label = (label, conf)
        cv2.putText(frame, f"Angle: {label}° {conf:.1f}%", (x1, y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,255), 2)
    return frame, crop, angle_label

# --- Main Routine ---
def main():
    cam1 = CameraThread(0)
    cam2 = CameraThread(1)
    cam1.start()
    cam2.start()

    # preload classifiers
    for pth in MODEL_CLASSIFY_PATHS:
        load_classify_model(pth)

    img_ctr = 1
    last_save = 0
    cooldown = 1.0

    print("Press 's' to save crops with angle labels, 'q' to quit.")
    while True:
        f1 = cam1.get_frame(); f2 = cam2.get_frame()
        if f1 is not None:
            out1, crop1, lbl1 = process_frame(f1, 1, MODEL_CLASSIFY_PATHS)
            cv2.imshow('Cam1', out1)
            if crop1 is not None:
                cv2.imshow('Crop1', crop1)
        if f2 is not None:
            out2, crop2, lbl2 = process_frame(f2, 2, MODEL_CLASSIFY_PATHS)
            cv2.imshow('Cam2', out2)
            if crop2 is not None:
                cv2.imshow('Crop2', crop2)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s') and time.time() - last_save >= cooldown:
            if crop1 is not None and lbl1:
                name = f"cam1_{lbl1[0]}deg_{img_ctr}.jpg"
                path = OUTPUT_DIR/name
                cv2.imwrite(str(path), crop1)
                print(f"Saved {path}")
            if crop2 is not None and lbl2:
                name = f"cam2_{lbl2[0]}deg_{img_ctr}.jpg"
                path = OUTPUT_DIR/name
                cv2.imwrite(str(path), crop2)
                print(f"Saved {path}")
            img_ctr += 1
            last_save = time.time()

    cam1.stop()
    cam2.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()