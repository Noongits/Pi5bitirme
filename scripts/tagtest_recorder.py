#!/usr/bin/env python3
import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys
import numpy as np
import time
from datetime import datetime

# Path to log file
log_path = "detections.txt"

# Initialize both cameras with separate instances
picam2   = Picamera2(camera_num=0)
picam2b  = Picamera2(camera_num=1)
debug    = True

# Create a preview configuration for each camera (same size)
conf1 = picam2.create_preview_configuration(main={"size": (1296, 972)})
conf2 = picam2b.create_preview_configuration(main={"size": (1296, 972)})

picam2.configure(conf1)
picam2b.configure(conf2)
picam2.set_controls({"FrameRate": 30})
picam2b.set_controls({"FrameRate": 30})

picam2.start()
picam2b.start()

# AprilTag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Intrinsics for camera 1 (picam2b) and camera 2 (picam2)
fx, fy, cx, cy = 1269, 1269, 1296/2, 972/2
K1 = np.array([[fx, 0, cx],[0, fy, cy],[0,0,1]], dtype=np.float32)
D1 = np.zeros((5,1))
fx2, fy2, cx2, cy2 = 1060, 1057, 1296/2, 972/2
K2 = np.array([[fx2,0,cx2],[0,fy2,cy2],[0,0,1]], dtype=np.float32)
D2 = np.zeros((5,1))

# Tag size and object points for solvePnP
tag_size = 0.084
hs = tag_size/2
objp = np.array([
    [-hs,  hs, 0],
    [ hs,  hs, 0],
    [ hs, -hs, 0],
    [-hs, -hs, 0]
], dtype=np.float32)

def signal_handler(sig, frame):
    print("Exiting...")
    picam2.stop()
    picam2b.stop()
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

frame_count = 0
start_time  = time.time()

while True:
    # --- CAMERA A ---
    frame  = picam2.capture_array()
    frame  = cv2.rotate(frame, cv2.ROTATE_180)
    gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray   = np.ascontiguousarray(gray)
    results = detector.detect(gray)

    if debug and results:
        for r in results:
            # draw box & center
            c = r.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(c[i]), tuple(c[(i+1)%4]), (0,255,0),2)
            ctr = tuple(r.center.astype(int))
            cv2.circle(frame, ctr, 5, (0,0,255), -1)
            cv2.putText(frame, f"ID:{r.tag_id}", (ctr[0]-20,ctr[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)

            # pose estimation
            imgp = np.array(r.corners, dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(objp, imgp, K1, D1)
            if ok:
                # annotate tvec
                tv = tvec.ravel()
                txt = f"t: {tv[0]:.3f},{tv[1]:.3f},{tv[2]:.3f}"
                cv2.putText(frame, txt, (ctr[0]-100, ctr[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

                # log to file
                ts = datetime.now().isoformat()
                with open(log_path, "a") as f:
                    f.write(f"{ts},Camera A,ID:{r.tag_id},tvec:{tv.tolist()}\n")

    cv2.putText(frame, "Camera A", (20,30),
                cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)

    # --- CAMERA B ---
    frame2   = picam2b.capture_array()
    frame2   = cv2.rotate(frame2, cv2.ROTATE_180)
    gray2    = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    gray2    = np.ascontiguousarray(gray2)
    results2 = detector.detect(gray2)

    if debug and results2:
        for r in results2:
            c = r.corners.astype(int)
            for i in range(4):
                cv2.line(frame2, tuple(c[i]), tuple(c[(i+1)%4]), (0,255,0),2)
            ctr = tuple(r.center.astype(int))
            cv2.circle(frame2, ctr, 5, (0,0,255), -1)
            cv2.putText(frame2, f"ID:{r.tag_id}", (ctr[0]-20,ctr[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)

            imgp = np.array(r.corners, dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(objp, imgp, K2, D2)
            if ok:
                tv = tvec.ravel()
                txt = f"t: {tv[0]:.3f},{tv[1]:.3f},{tv[2]:.3f}"
                cv2.putText(frame2, txt, (ctr[0]-100, ctr[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

                ts = datetime.now().isoformat()
                with open(log_path, "a") as f:
                    f.write(f"{ts},Camera B,ID:{r.tag_id},tvec:{tv.tolist()}\n")

    cv2.putText(frame2, "Camera B", (20,30),
                cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)

    # Show combined
    combined = cv2.hconcat([frame, frame2])
    cv2.imshow("Combined AprilTag Detection", combined)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    # FPS logging (optional)
    frame_count += 1
    now = time.time()
    if now - start_time >= 1.0:
        print(f"FPS: {frame_count / (now - start_time):.2f}")
        frame_count = 0
        start_time  = now

picam2.stop()
picam2b.stop()
cv2.destroyAllWindows()
