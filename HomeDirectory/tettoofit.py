#!/usr/bin/env python3
import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys
import numpy as np
import time

# Baseline: distance between cameras along X (5 cm)
baseline = np.array([0.05, 0.0, 0.0], dtype=np.float32)

# Initialize both cameras
picam2 = Picamera2(camera_num=0)
picam2num2 = Picamera2(camera_num=1)
debug = True

# Preview configurations
device_size = (1280, 960)
preview_config1 = picam2.create_preview_configuration(main={"size": device_size})
preview_config2 = picam2num2.create_preview_configuration(main={"size": device_size})

picam2.configure(preview_config1)
picam2num2.configure(preview_config2)
picam2.set_controls({"FrameRate": 30})
picam2num2.set_controls({"FrameRate": 30})

picam2.start()
picam2num2.start()

# AprilTag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Camera 0 intrinsics
fx0, fy0 = 1270.0, 1270.0
cx0, cy0 = device_size[0]/2, device_size[1]/2
camera_matrix0 = np.array([[fx0, 0, cx0], [0, fy0, cy0], [0, 0, 1]], dtype=np.float32)
dist_coeffs0 = np.zeros((5,1), dtype=np.float32)

# Camera 1 intrinsics
fx1, fy1 = 1060.0, 1057.0
cx1, cy1 = device_size[0]/2, device_size[1]/2
camera_matrix1 = np.array([[fx1, 0, cx1], [0, fy1, cy1], [0, 0, 1]], dtype=np.float32)
dist_coeffs1 = np.zeros((5,1), dtype=np.float32)

# Tag geometry
tag_size = 0.08  # meters
half = tag_size / 2.0
object_points = np.array([
    [-half,  half, 0], [ half,  half, 0],
    [ half, -half, 0], [-half, -half, 0]
], dtype=np.float32)

# Graceful exit
def signal_handler(sig, frame):
    picam2.stop(); picam2num2.stop(); cv2.destroyAllWindows(); sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# FPS tracking
start_time = time.time(); frame_count = 0

while True:
    poses0, poses1 = {}, {}

    # Camera 0
    frame0 = cv2.rotate(picam2.capture_array(), cv2.ROTATE_180)
    gray0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)
    for r in detector.detect(np.ascontiguousarray(gray0)):
        corners = r.corners.astype(int)
        for i in range(4): cv2.line(frame0, tuple(corners[i]), tuple(corners[(i+1)%4]), (0,255,0),2)
        cx, cy = int(r.center[0]), int(r.center[1])
        cv2.circle(frame0, (cx,cy),5,(0,0,255),-1)
        cv2.putText(frame0, f"ID:{r.tag_id}",(cx-20,cy-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
        ok,rvec,tvec = cv2.solvePnP(object_points, np.array(r.corners, np.float32), camera_matrix0, dist_coeffs0)
        if ok: poses0[r.tag_id] = tvec.ravel()
    cv2.putText(frame0, "Camera 0", (20,30), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)

    # Camera 1
    frame1 = cv2.rotate(picam2num2.capture_array(), cv2.ROTATE_180)
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    for r in detector.detect(np.ascontiguousarray(gray1)):
        corners = r.corners.astype(int)
        for i in range(4): cv2.line(frame1, tuple(corners[i]), tuple(corners[(i+1)%4]), (0,255,0),2)
        cx2, cy2 = int(r.center[0]), int(r.center[1])
        cv2.circle(frame1, (cx2,cy2),5,(0,0,255),-1)
        cv2.putText(frame1, f"ID:{r.tag_id}",(cx2-20,cy2-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
        ok,rvec,tvec = cv2.solvePnP(object_points, np.array(r.corners, np.float32), camera_matrix1, dist_coeffs1)
        if ok: poses1[r.tag_id] = tvec.ravel()
    cv2.putText(frame1, "Camera 1", (20,30), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)

    # Display
    combined = cv2.hconcat([frame0, frame1])

    # Show per-camera and fused positions
    for idx, tag_id in enumerate(sorted(set(poses0)&set(poses1))):
        t0 = poses0[tag_id]
        t1 = poses1[tag_id]
        t1_cam0 = t1 + baseline
        t_corr = (t0 + t1_cam0) * 0.5

        base_y = 50 + idx*60
        cv2.putText(combined, f"ID:{tag_id} C0: {t0.round(3)} m", (20, base_y), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),2)
        cv2.putText(combined, f"      C1: {t1.round(3)} m", (20, base_y+20), cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
        cv2.putText(combined, f"      F:  {t_corr.round(3)} m", (20, base_y+40), cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)

    cv2.imshow("Stereo AprilTag", combined)
    if cv2.waitKey(1)&0xFF==ord('q'): break

    frame_count+=1
    now=time.time()
    if now-start_time>=1.0:
        print(f"FPS: {frame_count/(now-start_time):.2f}")
        frame_count=0; start_time=now

picam2.stop(); picam2num2.stop(); cv2.destroyAllWindows()
