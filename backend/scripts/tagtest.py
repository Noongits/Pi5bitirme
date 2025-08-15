#!/usr/bin/env python3
import cv2
import apriltag
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
preview_config1 = picam2.create_preview_configuration(main={"size": (1296, 972)})
preview_config2 = picam2num2.create_preview_configuration(main={"size": (1296, 972)})


picam2.configure(preview_config1)
picam2num2.configure(preview_config2)
picam2.set_controls({"FrameRate": 30})
picam2num2.set_controls({"FrameRate": 30})

picam2.start()
picam2num2.start()

# Initialize the AprilTag detector (using the desired tag family)
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Camera intrinsics (example values; adjust if necessary)
fx = 1269
fy = 1269
cx = 1296 / 2
cy = 972 / 2
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))

fx2 = 1060
fy2 = 1057
cx2 = 1296 / 2
cy2 = 972 / 2
camera_matrix2 = np.array([[fx2, 0, cx2],
                          [0, fy2, cy2],
                          [0,  0,  1]], dtype=np.float32)
dist_coeffs2 = np.zeros((5, 1))

# Define tag physical size and corresponding object points for pose estimation
tag_size = 0.084
#tag_size = 0.078
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
    cv2.destroyAllWindows()
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
    
    results = detector.detect(gray)
    if debug:
        for r in results:
            corners = r.corners.astype(int)
            cv2.line(frame, tuple(corners[0]), tuple(corners[1]), (0, 255, 0), 2)
            cv2.line(frame, tuple(corners[1]), tuple(corners[2]), (0, 255, 0), 2)
            cv2.line(frame, tuple(corners[2]), tuple(corners[3]), (0, 255, 0), 2)
            cv2.line(frame, tuple(corners[3]), tuple(corners[0]), (0, 255, 0), 2)
            
            center = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {r.tag_id}", (center[0]-20, center[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            image_points = np.array(r.corners, dtype=np.float32)
            

            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix2, dist_coeffs2)
            if retval:
                tvec_str = f"t: {tvec.ravel()}"
                cv2.putText(frame, tvec_str, (center[0]-100, center[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                rot_mat, _ = cv2.Rodrigues(rvec)
    
    # Write camera name on first frame
    cv2.putText(frame, "Camera 2.1", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Process second camera
    frame2 = picam2num2.capture_array()
    frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray2 = np.ascontiguousarray(gray2)
    
    results2 = detector.detect(gray2)
    if debug:
        for r in results2:
            corners = r.corners.astype(int)
            cv2.line(frame2, tuple(corners[0]), tuple(corners[1]), (0, 255, 0), 2)
            cv2.line(frame2, tuple(corners[1]), tuple(corners[2]), (0, 255, 0), 2)
            cv2.line(frame2, tuple(corners[2]), tuple(corners[3]), (0, 255, 0), 2)
            cv2.line(frame2, tuple(corners[3]), tuple(corners[0]), (0, 255, 0), 2)
            
            center = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame2, center, 5, (0, 0, 255), -1)
            cv2.putText(frame2, f"ID: {r.tag_id}", (center[0]-20, center[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            if retval:
                tvec_str = f"t: {tvec.ravel()}"
                cv2.putText(frame2, tvec_str, (center[0]-100, center[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                rot_mat, _ = cv2.Rodrigues(rvec)
    
    # Write camera name on second frame
    cv2.putText(frame2, "Camera 1.3", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Combine the two frames side by side
    combined_frame = cv2.hconcat([frame, frame2])
    
    
    cv2.imshow("Combined AprilTag Detection", combined_frame)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    # Calculate and print FPS every second
    frame_count += 1
    current_time = time.time()
    elapsed_time = current_time - start_time
    if elapsed_time >= 1.0:
        fps = frame_count / elapsed_time
        print(f"FPS: {fps:.2f}")
        frame_count = 0
        start_time = current_time

picam2.stop()
picam2num2.stop()
cv2.destroyAllWindows()
