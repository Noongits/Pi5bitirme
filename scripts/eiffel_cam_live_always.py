from ultralytics import YOLO
import cv2
from pathlib import Path
from picamera2 import Picamera2
import threading
import queue

# Paths
model_path = 'models/bestbest_ncnn_model'
output_dir = Path('dataset/webcam_crops')
output_dir.mkdir(exist_ok=True)

# Load YOLO model
model = YOLO(model_path, task='detect')
model.conf = 0.8  # Confidence threshold

class CameraThread:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.picam2 = Picamera2(camera_id)
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = False
        
        config = self.picam2.create_preview_configuration(
            main={"size": (1280, 960), "format": "RGB888"},
            lores={"size": (640, 480), "format": "YUV420"}
        )
        self.picam2.configure(config)
        
    def start(self):
        self.running = True
        self.picam2.start()
        threading.Thread(target=self._capture_frames, daemon=True).start()
        
    def stop(self):
        self.running = False
        self.picam2.stop()
        
    def _capture_frames(self):
        while self.running:
            frame = self.picam2.capture_array()
            if not self.frame_queue.empty():
                self.frame_queue.get_nowait()
            self.frame_queue.put(frame)
            
    def get_frame(self):
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

def process_frame(frame, camera_id):
    # Rotate for correct orientation
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    results = model(frame)
    boxes = results[0].boxes
    if not boxes:
        return frame, None

    # Pick highest-confidence box
    best_box = max(boxes, key=lambda b: b.conf[0].item())
    x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())

    # Extract crop without drawing any overlays
    crop = frame[y1:y2, x1:x2]
    return frame, crop

def main():
    cam1 = CameraThread(0)
    cam2 = CameraThread(1)
    cam1.start()
    cam2.start()

    img_counter = 1
    print("\nDual Camera detection started! Press 'q' to quit.")

    try:
        while True:
            f1 = cam1.get_frame()
            f2 = cam2.get_frame()

            if f1 is not None:
                frame1, crop1 = process_frame(f1, 1)
                cv2.imshow('Camera 1', frame1)
                if crop1 is not None:
                    path1 = output_dir / f'cam1_crop_{img_counter:04d}.jpg'
                    #cv2.imwrite(str(path1), crop1)
                    cv2.imshow('crop', crop1)
                    #cv2.imwrite(str(path1), crop1)
                    print(f"Saved {path1}")
                    img_counter += 1

            if False and f2 is not None:
                frame2, crop2 = process_frame(f2, 2)
                cv2.imshow('Camera 2', frame2)
                if crop2 is not None:
                    path2 = output_dir / f'cam2_crop_{img_counter:04d}.jpg'
                    cv2.imwrite(str(path2), crop2)
                    
                    print(f"Saved {path2}")
                    img_counter += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam1.stop()
        cam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()