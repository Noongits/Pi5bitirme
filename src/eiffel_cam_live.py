from ultralytics import YOLO
import cv2
from pathlib import Path
import time
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
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
        self.thread = None
        
        # Configure camera
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            lores={"size": (320, 240), "format": "YUV420"}
        )
        self.picam2.configure(config)
        
    def start(self):
        self.running = True
        self.picam2.start()
        self.thread = threading.Thread(target=self._capture_frames)
        self.thread.start()
        
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.picam2.stop()
        
    def _capture_frames(self):
        while self.running:
            frame = self.picam2.capture_array()
            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self.frame_queue.put(frame)
            
    def get_frame(self):
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

def process_frame(frame, img_counter, camera_id):
    if frame is None:
        return None, img_counter, None
        
    try:
        results = model(frame)
        boxes = results[0].boxes
        if not boxes:
            return frame, img_counter, None

        # Find box with highest confidence
        best_box = max(boxes, key=lambda b: b.conf[0].item())

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
        conf = best_box.conf[0].item()
        cls = best_box.cls[0].item()

        # Draw rectangle on frame
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f'Cam{camera_id} Conf: {conf:.2f}', (int(x1), int(y1)-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Get the crop
        crop = frame[int(y1):int(y2), int(x1):int(x2)]
        
        return frame, img_counter, crop
    except Exception as e:
        print(f"Error processing frame from camera {camera_id}: {str(e)}")
        return frame, img_counter, None

def main():
    # Initialize cameras
    try:
        cam1 = CameraThread(0)  # First camera
        cam2 = CameraThread(1)  # Second camera
        
        cam1.start()
        cam2.start()
    except Exception as e:
        print(f"Error initializing cameras: {str(e)}")
        return

    img_counter = 1
    last_save_time = 0
    save_cooldown = 1.0  # Minimum seconds between saves

    print("\nDual Camera detection started!")
    print("Press 's' to save the current crop from both cameras")
    print("Press 'q' to quit")

    try:
        while True:
            # Get frames from both cameras
            frame1 = cam1.get_frame()
            frame2 = cam2.get_frame()
            
            if frame1 is not None:
                frame1, img_counter, crop1 = process_frame(frame1, img_counter, 1)
                cv2.imshow('Camera 1', frame1)
                if crop1 is not None:
                    cv2.imshow('Crop 1', crop1)
                    
            if frame2 is not None:
                frame2, img_counter, crop2 = process_frame(frame2, img_counter, 2)
                cv2.imshow('Camera 2', frame2)
                if crop2 is not None:
                    cv2.imshow('Crop 2', crop2)

            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                current_time = time.time()
                if current_time - last_save_time >= save_cooldown:
                    # Save crops from both cameras if available
                    if crop1 is not None:
                        crop_path1 = output_dir / f'cam1_crop_{img_counter}.jpg'
                        cv2.imwrite(str(crop_path1), crop1)
                        print(f"Saved crop from camera 1: {crop_path1}")
                        
                    if crop2 is not None:
                        crop_path2 = output_dir / f'cam2_crop_{img_counter}.jpg'
                        cv2.imwrite(str(crop_path2), crop2)
                        print(f"Saved crop from camera 2: {crop_path2}")
                        
                    if crop1 is not None or crop2 is not None:
                        img_counter += 1
                        last_save_time = current_time

    finally:
        # Cleanup
        cam1.stop()
        cam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 