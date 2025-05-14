import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys
import numpy as np
import time
import variables
from motor_controller import *

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

# Initialize the AprilTag detector (using the desired tag family)
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Camera intrinsics (example values; adjust if necessary)
fx = 1270
fy = 1270
cx = 1280 / 2
cy = 960 / 2
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))

fx2 = 1057
fy2 = 1057
cx2 = 1280 / 2
cy2 = 960 / 2
camera_matrix2 = np.array([[fx2, 0, cx2],
                           [0, fy2, cy2],
                           [0,  0,  1]], dtype=np.float32)
dist_coeffs2 = np.zeros((5, 1))

# Define tag physical size and corresponding object points for pose estimation
tag_size = 0.084
half_size = tag_size / 2.0
object_points = np.array([
    [-half_size,  half_size, 0],
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

def signal_handler(sig, frame):
    print("Exiting...")
    picam2.stop()
    picam2num2.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Variables for FPS calculation
start_time = time.time()
frame_count = 0


def detect_apriltag():
    while True:
        
        if not variables.calibrated:
            continue
        tagarray = np.zeros((15+1, 3), dtype=float)
        
        global frame_count
        global start_time
    
        if variables.calibrated:
            print(f"\n frame {frame_count} :")
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = np.ascontiguousarray(gray)
        
        results = detector.detect(gray)
        if debug and results:
            if variables.calibrated:
                print("Camera 2.1 detected tags:")
        for r in results:
            #print(f"ID: {r.tag_id}, Center: {r.center}")
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix2, dist_coeffs2)
            if retval:
                if variables.calibrated:
                    #print(f"ID: {r.tag_id} Pose: rvec: {rvec.ravel()}, tvec: {tvec.ravel()}")
                    idx = int(r.tag_id)
                    vec = tvec.ravel()      
                            # if there's already a non‑zero entry, average it with the new vec
                    if np.any(tagarray[idx, :] != 0):
                        tagarray[idx, :] = (tagarray[idx, :] + vec) / 2
                    else:
                        tagarray[idx, :] = vec
        
        # Process second camera
        frame2 = picam2num2.capture_array()
        frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        gray2 = np.ascontiguousarray(gray2)
        
        results2 = detector.detect(gray2)
        if debug and results2:
                if variables.calibrated:
                    print("Camera 1.3 detected tags:")
        for r in results2:
            #print(f"ID: {r.tag_id}, Center: {r.center}, ")
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            if retval:
                    if variables.calibrated:
                        #print(f"ID: {r.tag_id}, Pose: rvec: {rvec.ravel()}, tvec: {tvec.ravel()}")
                        idx = int(r.tag_id)
                        vec = tvec.ravel()            # shape (3,)
                        # if there's already a non‑zero entry, average it with the new vec
                        if np.any(tagarray[idx, :] != 0):
                            tagarray[idx, :] = (tagarray[idx, :] + vec) / 2
                        else:
                            tagarray[idx, :] = vec
                
    
        
        # Calculate and print FPS every second
        if variables.calibrated:
            #print(tagarray)
            variables.tagarray = tagarray
            print(tagarray[0, 2])
            if tagarray[0, 2] > 3.5 and variables.currentlyForward == False:
                move_forward()
            if tagarray[0, 2] <= 3.5 and variables.currentlyForward:
                stop_motors
            
            
        frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            if variables.calibrated:
                print(f"FPS: {fps:.2f}")
            frame_count = 0
            start_time = current_time