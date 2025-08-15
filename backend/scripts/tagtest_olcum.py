#!/usr/bin/env python3
import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys
import numpy as np
import time

# --- Configuration ---

# Camera instances
cam1 = Picamera2(camera_num=0)
cam2 = Picamera2(camera_num=1)



# Same preview size for both
config1 = cam1.create_preview_configuration(main={"size": (1296, 972)})
config2 = cam2.create_preview_configuration(main={"size": (1296, 972)})

for cam, cfg in ((cam1, config1), (cam2, config2)):
    cam.configure(cfg)
    cam.set_controls({"FrameRate": 30})
    cam.start()

# AprilTag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Example intrinsics for pose estimation (tweak as needed)
fx, fy = 1269, 1269
cx, cy = 1296 / 2, 972 / 2
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]], dtype=np.float32)
dist = np.zeros((5, 1))

# Tag size for pose (meters)
tag_size = 0.084
h = tag_size / 2.0
obj_pts = np.array([
    [-h,  h, 0],
    [ h,  h, 0],
    [ h, -h, 0],
    [-h, -h, 0],
], dtype=np.float32)

def cleanup_and_exit(signum, frame):
    print("\nShutting down...")
    for cam in (cam1, cam2):
        cam.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup_and_exit)

# --- Main loop ---
start = time.time()
frames = 0

while True:
    for name, cam in (("Cam 0", cam1), ("Cam 1", cam2)):
        img = cam.capture_array()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = np.ascontiguousarray(gray)
        print("detecting")
        results = detector.detect(gray)
        if results:
            print(f"[{name}] {len(results)} tag(s) detected:")
            for r in results:
                print(f"  • ID={r.tag_id}")
                print(f"    corners={np.round(r.corners,1).tolist()}")
                print(f"    center={tuple(np.round(r.center,1))}")
                # Pose estimation
                pts = r.corners.astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(obj_pts, pts, K, dist, flags=cv2.SOLVEPNP_IPPE)
                if success:
                    t = tuple(np.round(tvec.ravel(), 3))
                    print(f"    translation (m): {t}")
            print()

    # FPS calc every second
    frames += 1
    now = time.time()
    if now - start >= 1.0:
        print(f"Overall FPS: {frames / (now - start):.1f}\n")
        frames = 0
        start = now
