from ultralytics import YOLO
import cv2
from pathlib import Path
from picamera2 import Picamera2, Preview
import threading
import queue

# Paths
model_path = 'models/bestbest_ncnn_model'
output_dir = Path('dataset/webcam_crops_new')
output_dir.mkdir(exist_ok=True)

# Load YOLO model
model = YOLO(model_path, task='detect')
model.conf = 0.8  # Confidence threshold

class CameraThread:
    def __init__(self, camera_id, mode='video', resolution=(1920, 1080)):
        """
        mode: 'video' for continuous HD, 'still' for max-res captures
        resolution: tuple width×height
        """
        self.camera_id = camera_id
        self.picam2 = Picamera2(camera_num=camera_id)
        self.frame_queue = queue.Queue(maxsize=2)
        self.running = False

        if mode == 'video':
            # Full-HD video stream (might go up to 1080p@30fps)
            config = self.picam2.create_video_configuration(
                main={"size": resolution, "format": "RGB888"}
            )
        else:
            # Maximum still‐image resolution (3280×2464 on v2.1)
            config = self.picam2.create_still_configuration(
                {"size": resolution, "format": "RGB888"}
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

def process_frame(frame):
    # Rotate for correct orientation
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    results = model(frame)
    boxes = results[0].boxes
    if not boxes:
        return None

    # Pick highest-confidence box
    best_box = max(boxes, key=lambda b: b.conf[0].item())
    x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())

    # Extract crop
    return frame[y1:y2, x1:x2]

def main():
    # Example: use Full-HD (1920×1080). For max still res, pass mode='still', resolution=(3280,2464)
    cam1 = CameraThread(0, mode='video', resolution=(1920,1080))
    cam2 = CameraThread(1, mode='video', resolution=(1920,1080))
    cam1.start()
    cam2.start()

    img_counter = 1
    print("\nDual Camera detection started! Processing high-res frames...")

    try:
        while True:
            for cam, prefix in ((cam1, 'cam1'), (cam2, 'cam2')):
                frame = cam.get_frame()
                if frame is None:
                    continue
                crop = process_frame(frame)
                if crop is not None:
                    path = output_dir / f'{prefix}_crop_{img_counter:04d}.jpg'
                    cv2.imwrite(str(path), crop)
                    print(f"Saved {path}")
                    img_counter += 1
    finally:
        cam1.stop()
        cam2.stop()

if __name__ == "__main__":
    main()
