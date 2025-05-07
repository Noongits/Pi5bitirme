import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys
import numpy as np
import time
import state
from motor_controller import *

# --- CONFIG ---

# Hardcoded AprilTag world coordinates (in meters)
APRILTAG_COORDS = {
    0: np.array([0.0, 0.0, 4.0]),
    2: np.array([-2.1, 2.55, 0.0]),
    3: np.array([5.0, 0.0, 0.0]),
    6: np.array([0.0, -5.0, 0.0]),
    8: np.array([-5.0, 0.0, 0.0])
}

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

# Initialize variables
car_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
stage = 0  # 0: forward, 1: turn, 2: final leg

# Destination input
destination = np.array([
    float(input("Enter destination X (meters): ")),
    float(input("Enter destination Y (meters): ")),
    0.0
])

# Setup cameras
picam2 = Picamera2(camera_num=0)
picam2num2 = Picamera2(camera_num=1)
preview_config1 = picam2.create_preview_configuration(main={"size": (1280, 960)})
preview_config2 = picam2num2.create_preview_configuration(main={"size": (1280, 960)})
picam2.configure(preview_config1)
picam2num2.configure(preview_config2)
picam2.set_controls({"FrameRate": 30})
picam2num2.set_controls({"FrameRate": 30})
picam2.start()
picam2num2.start()

# Detector
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Exit handler
def signal_handler(sig, frame):
    print("Exiting...")
    stop_motors()
    picam2.stop()
    picam2num2.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Main loop
def detect_apriltag():
    global stage, car_pose

    frame_count = 0
    start_time = time.time()

    while True:
        if not state.calibrated:
            continue

        tagarray = np.zeros((15+1, 3), dtype=float)

        # --- CAMERA 1 ---
        
        frame = picam2.capture_array()
        with state.lock:
            state.currentframe = frame
        
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

        for r in results:
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix2, dist_coeffs2)
            if retval:
                idx = int(r.tag_id)
                vec = tvec.ravel()
                if np.any(tagarray[idx, :] != 0):
                    tagarray[idx, :] = (tagarray[idx, :] + vec) / 2
                else:
                    tagarray[idx, :] = vec

        # --- CAMERA 2 ---
        frame2 = picam2num2.capture_array()
        frame2 = cv2.rotate(frame2, cv2.ROTATE_180)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
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

        state.tagarray = tagarray

        # Pose estimation
        detected_tags = [tid for tid in APRILTAG_COORDS if np.any(tagarray[tid] != 0)]
        if not detected_tags:
            #print("No known tags detected.")
            stop_motors()
            continue

        nearest_tag = min(detected_tags, key=lambda tid: np.linalg.norm(tagarray[tid]))
        relative_pos = tagarray[nearest_tag]
        tag_world = APRILTAG_COORDS[nearest_tag]
        car_position_est = tag_world - relative_pos
        car_pose = car_position_est

        print(f"Estimated position: {car_pose}  Tag World: {tag_world} Tag reletive: {relative_pos}")

        # STAGE 0: Move forward
        if stage == 0 and NavMesh:
            distance = np.linalg.norm(relative_pos)
            print(f"To tag {nearest_tag}: {distance:.2f} m")
            if distance > 2.5:
                move_forward()
                state.currentlyForward = True
            else:
                stop_motors()
                state.currentlyForward = False
                print("Close to tag. Preparing to turn...")
                stage = 1

        # STAGE 1: Turn left or right
        elif stage == 1 and NavMesh:
            dx = destination[0] - car_pose[0]
            direction = "left" if dx < 0 else "right"
            print(f"Turning {direction}...")
            if direction == "left":
                turn_left()
            else:
                turn_right()
            time.sleep(1.5)  # Adjust for 90-degree turn
            stop_motors()
            stage = 2

        # STAGE 2: Move toward destination
        elif stage == 2 and NavMesh:
            dist_to_dest = np.linalg.norm(destination[:2] - car_pose[:2])
            print(f"Distance to destination: {dist_to_dest:.2f} m")
            if dist_to_dest > 0.5:
                move_forward()
                state.currentlyForward = True
            else:
                stop_motors()
                print("Destination reached.")
                break

        # FPS print
        frame_count += 1
        current_time = time.time()
        if current_time - start_time >= 1.0:
            fps = frame_count / (current_time - start_time)
            print(f"FPS: {fps:.2f}")
            frame_count = 0
            start_time = current_time

