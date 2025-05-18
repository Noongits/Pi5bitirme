import os
import cv2
import numpy as np
import signal
import sys
import time

from picamera2 import Picamera2
from ultralytics import YOLO
from motor_controller import stop_motors

# --- Debug flag: set to False to disable all windows and key handling ---
DEBUG = False

# --- Prepare output dirs ---
IMG_DIR = "images/train"
IMG_UN  = "images/unlabeled"
LBL_DIR = "labels/train"
os.makedirs(IMG_DIR, exist_ok=True)
os.makedirs(IMG_UN, exist_ok=True)
os.makedirs(LBL_DIR, exist_ok=True)

# --- Setup camera with 3-channel RGB output ---
picam1 = Picamera2(camera_num=0)
config1 = picam1.create_preview_configuration(
    main={"size": (1280, 960), "format": "RGB888"}
)
picam1.configure(config1)
picam1.set_controls({"FrameRate": 30})
picam1.start()

# --- Load YOLO model ---
model = YOLO("bestbest_ncnn_model")  # update path to your .pt file

# --- Clean exit handler ---
def signal_handler(sig, frame):
    print("Exiting...")
    stop_motors()
    picam1.stop()
    if DEBUG:
        cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# --- Main loop ---
def main():
    frame_count = 0
    start_time = time.time()

    while True:
        # Capture and rotate
        frame = picam1.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Run YOLO inference
        result  = model(frame)[0]
        boxes   = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()

        # Save detections (or unlabeled)
        ts = time.strftime("%Y%m%d-%H%M%S") + f"-{int(time.time()%1*1000):03d}"
        if len(boxes) > 0:
            img_path = os.path.join(IMG_DIR, f"{ts}.jpg")
            lbl_path = os.path.join(LBL_DIR, f"{ts}.txt")
            cv2.imwrite(img_path, frame)
            h, w = frame.shape[:2]
            with open(lbl_path, "w") as f:
                for (x1,y1,x2,y2), c in zip(boxes, classes):
                    x_c = ((x1+x2)/2)/w
                    y_c = ((y1+y2)/2)/h
                    bw  = (x2-x1)/w
                    bh  = (y2-y1)/h
                    f.write(f"{int(c)} {x_c:.6f} {y_c:.6f} {bw:.6f} {bh:.6f}\n")
        else:
            img_path = os.path.join(IMG_UN, f"{ts}.jpg")
            cv2.imwrite(img_path, frame)
            stop_motors()

        # Draw detections for visualization
        for (x1,y1,x2,y2), c in zip(boxes, classes):
            pt1 = (int(x1), int(y1))
            pt2 = (int(x2), int(y2))
            cv2.rectangle(frame, pt1, pt2, (0,255,0), 2)
            cv2.putText(frame, f"{int(c)}", (pt1[0], pt1[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

        # Only show window / read key if debugging
        if DEBUG:
            cv2.imshow("YOLO Cam1", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # FPS calculation
        frame_count += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            print(f"FPS: {frame_count/elapsed:.2f}")
            frame_count = 0
            start_time  = time.time()

    # Cleanup on exit
    picam1.stop()
    if DEBUG:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
