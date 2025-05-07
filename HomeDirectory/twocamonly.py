#!/usr/bin/env python3
import cv2
from picamera2 import Picamera2
import signal
import sys
import numpy as np
import time

# Initialize both cameras with separate instances
picam2 = Picamera2(camera_num=0)
picam2num2 = Picamera2(camera_num=1)
debug = True

# Create a preview configuration for each camera (using the same desired size)
preview_config1 = picam2.create_preview_configuration(main={"size": (1280, 960)})
preview_config2 = picam2num2.create_preview_configuration(main={"size": (1280, 960)})

picam2.configure(preview_config1)
picam2num2.configure(preview_config2)
picam2.set_controls({"FrameRate": 30})
picam2num2.set_controls({"FrameRate": 30})
picam2.start()
picam2num2.start()

def signal_handler(sig, frame):
    print("Exiting...")
    picam2.stop()
    picam2num2.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Variables for FPS calculation
start_time = time.time()
frame_count = 0

while True:
    # Process first camera
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = np.ascontiguousarray(gray)
    
    # Process second camera
    frame2 = picam2num2.capture_array()
    frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray2 = np.ascontiguousarray(gray2)
    
    # Calculate and print FPS every second
    frame_count += 1
    current_time = time.time()
    elapsed_time = current_time - start_time
    if elapsed_time >= 1.0:
        fps = frame_count / elapsed_time
        print(f"FPS: {fps:.2f}")
        frame_count = 0
        start_time = current_time
