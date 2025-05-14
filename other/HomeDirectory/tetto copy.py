#!/usr/bin/env python3
import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys
import numpy as np

# Initialize both cameras with separate instances
picam2 = Picamera2(camera_num=0)
picam2num2 = Picamera2(camera_num=1)

# Create a preview configuration for each camera (using the same desired size)
preview_config1 = picam2.create_preview_configuration(main={"size": (1280, 960)})
preview_config2 = picam2num2.create_preview_configuration(main={"size": (1280, 960)})

picam2.configure(preview_config1)
picam2num2.configure(preview_config2)

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

# Define tag physical size and corresponding object points for pose estimation
tag_size = 0.08
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

while True:
    # Process first camera
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Ensure the grayscale image is contiguous in memory
    gray = np.ascontiguousarray(gray)
    
    results = detector.detect(gray)
    
    for r in results:
        corners = r.corners.astype(int)
        # Draw bounding box around detected tag
        cv2.line(frame, tuple(corners[0]), tuple(corners[1]), (0, 255, 0), 2)
        cv2.line(frame, tuple(corners[1]), tuple(corners[2]), (0, 255, 0), 2)
        cv2.line(frame, tuple(corners[2]), tuple(corners[3]), (0, 255, 0), 2)
        cv2.line(frame, tuple(corners[3]), tuple(corners[0]), (0, 255, 0), 2)
        
        center = (int(r.center[0]), int(r.center[1]))
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        cv2.putText(frame, f"ID: {r.tag_id}", (center[0]-20, center[1]-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        image_points = np.array(r.corners, dtype=np.float32)
        retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        if retval:
            tvec_str = f"t: {tvec.ravel()}"
            cv2.putText(frame, tvec_str, (center[0]-100, center[1]+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # Convert rotation vector to rotation matrix (if needed)
            rot_mat, _ = cv2.Rodrigues(rvec)
    
    # Process second camera
    frame2 = picam2num2.capture_array()
    frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray2 = np.ascontiguousarray(gray2)
    
    results2 = detector.detect(gray2)
    
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
    
    # Display the processed frame from camera 2 (you may also show camera 1 if desired)
    
    combined_frame = cv2.hconcat([frame, frame2])
    cv2.imshow("Combined AprilTag Detection", combined_frame)
    
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
picam2num2.stop()
cv2.destroyAllWindows()
