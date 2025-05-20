import os
import numpy as np
import signal
import sys
import variables
import time

from PIL import Image
from picamera2 import Picamera2
from ultralytics import YOLO
from motor_controller import stop_motors

# --- Debug flag: set to False to disable all windows and key handling ---
DEBUG = False

if DEBUG:
    import matplotlib.pyplot as plt
    plt.ion()
    fig, ax = plt.subplots()
    exit_flag = {"stop": False}
    def on_key(event):
        if event.key == "q":
            exit_flag["stop"] = True
    fig.canvas.mpl_connect("key_press_event", on_key)

# --- Prepare output dirs ---
IMG_DIR = "images/train"
IMG_UN  = "images/unlabeled"
LBL_DIR = "labels/train"
os.makedirs(IMG_DIR, exist_ok=True)
os.makedirs(IMG_UN, exist_ok=True)
os.makedirs(LBL_DIR, exist_ok=True)

# --- Setup camera with 3-channel RGB output ---

# --- Load YOLO model ---
model = YOLO("bestbest_ncnn_model")  # update path to your .pt file

# --- Main loop ---
def main():
    frame_count = 0
    start_time = time.time()

    while True:
        # Capture latest frame under lock
        if variables.leftlock.acquire(blocking=True):
            frame = variables.leftcam
            variables.leftlock.release()

        if frame is not None:
            # Run YOLO inference
            result  = model(frame)[0]
            boxes   = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy()

            # Timestamp for filenames
            ts = time.strftime("%Y%m%d-%H%M%S") + f"-{int(time.time()%1*1000):03d}"

            # Save detections or unlabeled
            if len(boxes) > 0:
                print("DETECTED TOWER")
                
            # Debug display (matplotlib)
            if DEBUG:
                ax.clear()
                ax.imshow(frame)
                for (x1,y1,x2,y2), c in zip(boxes, classes):
                    w_box, h_box = x2 - x1, y2 - y1
                    rect = plt.Rectangle(
                        (x1, y1), w_box, h_box,
                        fill=False, edgecolor="green", linewidth=2
                    )
                    ax.add_patch(rect)
                    ax.text(x1, y1 - 8, str(int(c)),
                            fontsize=12, color="green")
                ax.axis("off")
                fig.canvas.draw()
                fig.canvas.flush_events()
                if exit_flag["stop"]:
                    break

            # FPS calculation
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                print(f"FPS: {frame_count/elapsed:.2f}")
                frame_count = 0
                start_time  = time.time()

            time.sleep(0.3)

        # Cleanup on exit
        if DEBUG:
            plt.ioff()
            plt.close(fig)

if __name__ == "__main__":
    main()
