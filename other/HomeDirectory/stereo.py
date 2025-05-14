#!/usr/bin/env python3
import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys
import numpy as np
import time

# --- 0. Setup and calibration ---

# Initialize both cameras
picam2     = Picamera2(camera_num=0)   # PiCam 2.1
picam2num2 = Picamera2(camera_num=1)   # PiCam 1.3

for cam in (picam2, picam2num2):
    cfg = cam.create_preview_configuration(main={"size": (1296, 972)})
    cam.configure(cfg)
    cam.set_controls({"FrameRate": 30})
    cam.start()

# AprilTag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Intrinsics (px)
fx,  fy,  cx,         cy         = 1060.0, 1057.0, 1296/2, 972/2    # PiCam2.1
fx2, fy2, cx2,        cy2        = 1269.0, 1269.0, 1296/2, 972/2    # PiCam1.3
K1 = np.array([[fx,  0, cx],
               [ 0, fy, cy],
               [ 0,  0,  1]], dtype=np.float64)
K2 = np.array([[fx2,  0, cx2],
               [  0, fy2, cy2],
               [  0,   0,   1]], dtype=np.float64)

# Stereo extrinsics (assume cameras are level, separated along X)
baseline = 0.10  # meters (adjust to your rig)
R = np.eye(3)
T = np.array([[baseline], [0.0], [0.0]], dtype=np.float64)

# Projection matrices P = K [R | –R·C], here C1 = [0,0,0], C2 = T
P1 = K1 @ np.hstack((np.eye(3), np.zeros((3,1))))
P2 = K2 @ np.hstack((R, -R @ T))

# Graceful exit
def signal_handler(sig, frame):
    picam2.stop(); picam2num2.stop()
    cv2.destroyAllWindows()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# FPS vars
start_time = time.time()
frame_count = 0

while True:
    # --- Capture + detect camera 1 (PiCam2.1) ---
    frame1 = picam2.capture_array()
    frame1 = cv2.rotate(frame1, cv2.ROTATE_180)
    gray1  = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    dets1  = detector.detect(gray1)

    for d in dets1:
        c = d.corners.astype(int)
        for i in range(4):
            cv2.line(frame1, tuple(c[i]), tuple(c[(i+1)%4]), (0,255,0), 2)
        u,v = int(d.center[0]), int(d.center[1])
        cv2.circle(frame1, (u,v), 5, (0,0,255), -1)
        cv2.putText(frame1, f"ID:{d.tag_id}", (u-20,v-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    cv2.putText(frame1, "Cam 2.1", (20,30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    # --- Capture + detect camera 2 (PiCam1.3) ---
    frame2 = picam2num2.capture_array()
    frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
    gray2  = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    dets2  = detector.detect(gray2)

    for d in dets2:
        c = d.corners.astype(int)
        for i in range(4):
            cv2.line(frame2, tuple(c[i]), tuple(c[(i+1)%4]), (0,255,0), 2)
        u,v = int(d.center[0]), int(d.center[1])
        cv2.circle(frame2, (u,v), 5, (0,0,255), -1)
        cv2.putText(frame2, f"ID:{d.tag_id}", (u-20,v-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    cv2.putText(frame2, "Cam 1.3", (20,30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    # --- Triangulate matching tags ---
    det1_by_id = {d.tag_id: d for d in dets1}
    det2_by_id = {d.tag_id: d for d in dets2}
    for tag_id in det1_by_id.keys() & det2_by_id.keys():
        d1 = det1_by_id[tag_id]
        d2 = det2_by_id[tag_id]
        # pixel centers
        pts1 = np.array([[d1.center[0]], [d1.center[1]]], dtype=np.float64)
        pts2 = np.array([[d2.center[0]], [d2.center[1]]], dtype=np.float64)
        # triangulate
        pts4d = cv2.triangulatePoints(P1, P2, pts1, pts2)
        pts3d = (pts4d[:3] / pts4d[3]).flatten()
        X, Y, Z = pts3d
        # annotate depth on frame1
        u,v = int(d1.center[0]), int(d1.center[1])
        cv2.putText(frame1, f"Z={Z:.2f}m", (u-50, v+40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

    # --- Show combined view ---
    combined = cv2.hconcat([frame1, frame2])
    cv2.imshow("Stereo AprilTag Triangulation", combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # FPS
    frame_count += 1
    if time.time() - start_time >= 1.0:
        print(f"FPS: {frame_count:.1f}")
        frame_count = 0
        start_time = time.time()

# cleanup
picam2.stop(); picam2num2.stop()
cv2.destroyAllWindows()
