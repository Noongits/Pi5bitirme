import cv2
import apriltag
import signal
import sys
import numpy as np
import time
import variables
from motor_controller import *

# Camera intrinsics
NavMesh = False
fx = 1270
fy = 1270
cx = 1280 / 2
cy = 960 / 2
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))

fx2 = 1057
fy2 = 1057
cx2 = 1280 / 2
cy2 = 960 / 2
camera_matrix2 = np.array([[fx2, 0, cx2], [0, fy2, cy2], [0, 0, 1]], dtype=np.float32)
dist_coeffs2 = np.zeros((5, 1))

tag_size = 0.08
half_size = tag_size / 2.0
object_points = np.array([
    [-half_size,  half_size, 0],
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

# Detector
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Exit handler
def signal_handler(sig, frame):
    print("Exiting...")
    stop_motors()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Main loop
def detect_apriltag():
    frame_count = 0
    start_time = time.time()

    while True:
        if not variables.calibrated:
            continue

        tagarray = np.zeros((16, 3), dtype=float)

        # --- CAMERA 1 ---
        # variables.tag_frame_left = cv2.rotate(variables.left_frame_imm, cv2.ROTATE_180)
        gray = cv2.cvtColor(variables.left_frame_imm, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

        for r in results:
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix2, dist_coeffs2)
            if retval:
                idx = int(r.tag_id)
                if idx > 15:
                    continue
                vec = tvec.ravel()
                if np.any(tagarray[idx, :] != 0):
                    tagarray[idx, :] = (tagarray[idx, :] + vec) / 2
                else:
                    tagarray[idx, :] = vec

        # --- CAMERA 2 ---
        # variables.tag_frame_right = cv2.rotate(variables.right_frame_imm, cv2.ROTATE_180)
        gray2 = cv2.cvtColor(variables.right_frame_imm, cv2.COLOR_BGR2GRAY)
        results2 = detector.detect(gray2)

        for r in results2:
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            if retval:
                idx = int(r.tag_id)
                vec = tvec.ravel()
                if np.any(tagarray[idx, :] != 0):
                    tagarray[idx, :] = (tagarray[idx, :] + vec) / 2
                else:
                    tagarray[idx, :] = vec

        variables.tagarray = tagarray

        variables.detected_tags = [tid for tid in variables.APRILTAG_COORDS if np.any(tagarray[tid] != 0)]

        # FPS print
        frame_count += 1
        current_time = time.time()
        if current_time - start_time >= 1.0:
            fps = frame_count / (current_time - start_time)
            print(f"FPS: {fps:.2f}")
            frame_count = 0
            start_time = current_time

        if variables.destination_reached:
            break
