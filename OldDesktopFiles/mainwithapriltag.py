# main.py
import time
import math
import threading
import mpu6050  # Ensure mpu6050.py is in the same directory or in your Python path

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Global variables for sensor data
estimated_position = [0.0, 0.0, 0.0]
calibrated = False
currentlyForward = False
currentlyBackward = False
currentlyRight = False
currentlyLeft = False

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
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Variables for FPS calculation
start_time = time.time()
frame_count = 0

def apriltag_loop():
    while True:
        
        tagarray = np.zeros((15+1, 3), dtype=float)
        global frame_count
        global start_time
        global calibrated
        global currentlyForward
    
        if calibrated:
            print(f"\n frame {frame_count} :")
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = np.ascontiguousarray(gray)
        
        results = detector.detect(gray)
        if debug and results:
            if calibrated:
                print("Camera 2.1 detected tags:")
        for r in results:
            #print(f"ID: {r.tag_id}, Center: {r.center}")
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix2, dist_coeffs2)
            if retval:
                if calibrated:
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
                if calibrated:
                    print("Camera 1.3 detected tags:")
        for r in results2:
            #print(f"ID: {r.tag_id}, Center: {r.center}, ")
            image_points = np.array(r.corners, dtype=np.float32)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            if retval:
                    if calibrated:
                        #print(f"ID: {r.tag_id}, Pose: rvec: {rvec.ravel()}, tvec: {tvec.ravel()}")
                        idx = int(r.tag_id)
                        vec = tvec.ravel()            # shape (3,)
                        # if there's already a non‑zero entry, average it with the new vec
                        if np.any(tagarray[idx, :] != 0):
                            tagarray[idx, :] = (tagarray[idx, :] + vec) / 2
                        else:
                            tagarray[idx, :] = vec
                
    
        
        # Calculate and print FPS every second
        if calibrated:
            #print(tagarray)
            print(tagarray[0, 2])
            if tagarray[0, 2] > 0.5 and currentlyForward == False:
                move_forward()
            if tagarray[0, 2] <= 0.5 and currentlyForward:
                stop_motors
            
            
        frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            if calibrated:
                print(f"FPS: {fps:.2f}")
            frame_count = 0
            start_time = current_time

    
def sensor_loop():
    """
    Initializes the sensor, performs calibration, and continuously reads sensor data
    to update the estimated_position variable.
    """
    # Initialize sensor and perform calibration
    mpu6050.mpu6050_init()
    ax_off, ay_off, az_off, gx_off, gy_off, gz_off = mpu6050.calibrate(num_samples=1000)
    global calibrated
    calibrated = True

    # Use monotonic time for reliable dt calculation
    prev_time = time.monotonic()
    velocity = [0.0, 0.0, 0.0]  # Velocity in m/s for x, y, z
    damping_coefficient = 0.5  # Damping coefficient to reduce integration drift
    global estimated_position
    estimated_position = [0.0, 0.0, 0.0]
    global currentlyForward


    try:
        while True:
            current_time = time.monotonic()
            dt = current_time - prev_time
            prev_time = current_time

            # Calculate damping factor to mitigate drift
            damping_factor = math.exp(-damping_coefficient * dt)

            # Read sensor data
            data = mpu6050.read_sensor_data()
            
            # Convert raw accelerometer values to m/s² (apply calibration offsets)
            ax = (data['ax'] - ax_off) / 16384.0 * 9.81
            ay = (data['ay'] - ay_off) / 16384.0 * 9.81
            az = (data['az'] - az_off) / 16384.0 * 9.81

            # Integrate acceleration to update velocity (with damping)
            if True:
                velocity[0] = (velocity[0] + ax * dt) * damping_factor
                velocity[1] = (velocity[1] + ay * dt) * damping_factor
                velocity[2] = (velocity[2] + az * dt) * damping_factor
            else:
                velocity[0] += ax * dt
                velocity[1] += ay * dt
                velocity[2] += az * dt
                # Integrate velocity to update the estimated position
            
            
            
            estimated_position[0] += velocity[0] * dt
            estimated_position[1] += velocity[1] * dt
            if False:
                estimated_position[2] += velocity[2] * dt

            time.sleep(0.00001)
    except KeyboardInterrupt:
        print("Sensor loop interrupted. Exiting sensor thread.")


# ------------------ Control Server Code ------------------
from flask import Flask, redirect, url_for
from flask_cors import CORS
import RPi.GPIO as GPIO


app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# --- GPIO and Motor Setup ---
# Define the GPIO pins (using BCM numbering)
# Back motor pins
BACK_LEFT_PIN1 = 5
BACK_LEFT_PIN2 = 6
BACK_RIGHT_PIN1 = 1
BACK_RIGHT_PIN2 = 7

# Front motor pins
FRONT_LEFT_PIN1 = 26
FRONT_LEFT_PIN2 = 21
FRONT_RIGHT_PIN1 = 20
FRONT_RIGHT_PIN2 = 16

# Set up GPIO
GPIO.setmode(GPIO.BCM)

# Setup for front motors
GPIO.setup(FRONT_LEFT_PIN1, GPIO.OUT)
GPIO.setup(FRONT_LEFT_PIN2, GPIO.OUT)
GPIO.setup(FRONT_RIGHT_PIN1, GPIO.OUT)
GPIO.setup(FRONT_RIGHT_PIN2, GPIO.OUT)

# Setup for back motors
GPIO.setup(BACK_LEFT_PIN1, GPIO.OUT)
GPIO.setup(BACK_LEFT_PIN2, GPIO.OUT)
GPIO.setup(BACK_RIGHT_PIN1, GPIO.OUT)
GPIO.setup(BACK_RIGHT_PIN2, GPIO.OUT)

# ----- FRONT Motor Functions -----
def FRONT_left_motor_forward():
    GPIO.output(FRONT_LEFT_PIN1, GPIO.HIGH)
    GPIO.output(FRONT_LEFT_PIN2, GPIO.LOW)

def FRONT_left_motor_backward():
    GPIO.output(FRONT_LEFT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_LEFT_PIN2, GPIO.HIGH)

def FRONT_left_motor_stop():
    GPIO.output(FRONT_LEFT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_LEFT_PIN2, GPIO.LOW)

def FRONT_right_motor_forward():
    GPIO.output(FRONT_RIGHT_PIN1, GPIO.HIGH)
    GPIO.output(FRONT_RIGHT_PIN2, GPIO.LOW)

def FRONT_right_motor_backward():
    GPIO.output(FRONT_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_RIGHT_PIN2, GPIO.HIGH)

def FRONT_right_motor_stop():
    GPIO.output(FRONT_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_RIGHT_PIN2, GPIO.LOW)

# ----- BACK Motor Functions -----
def BACK_left_motor_forward():
    GPIO.output(BACK_LEFT_PIN1, GPIO.HIGH)
    GPIO.output(BACK_LEFT_PIN2, GPIO.LOW)

def BACK_left_motor_backward():
    GPIO.output(BACK_LEFT_PIN1, GPIO.LOW)
    GPIO.output(BACK_LEFT_PIN2, GPIO.HIGH)

def BACK_left_motor_stop():
    GPIO.output(BACK_LEFT_PIN1, GPIO.LOW)
    GPIO.output(BACK_LEFT_PIN2, GPIO.LOW)

def BACK_right_motor_forward():
    GPIO.output(BACK_RIGHT_PIN1, GPIO.HIGH)
    GPIO.output(BACK_RIGHT_PIN2, GPIO.LOW)

def BACK_right_motor_backward():
    GPIO.output(BACK_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(BACK_RIGHT_PIN2, GPIO.HIGH)

def BACK_right_motor_stop():
    GPIO.output(BACK_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(BACK_RIGHT_PIN2, GPIO.LOW)

# --- Higher-Level Movement Functions ---
def stop_motors():
    FRONT_left_motor_stop()
    FRONT_right_motor_stop()
    BACK_left_motor_stop()
    BACK_right_motor_stop()

def move_forward(duration=1):
    global currentlyForward
    FRONT_left_motor_forward()
    FRONT_right_motor_forward()
    BACK_left_motor_forward()
    BACK_right_motor_forward()
    currentlyForward = True
    time.sleep(duration)
    stop_motors()
    currentlyForward = False

def move_backward(duration=1):
    FRONT_left_motor_backward()
    FRONT_right_motor_backward()
    BACK_left_motor_backward()
    BACK_right_motor_backward()
    time.sleep(duration)
    stop_motors()

def turn_left(duration=1):
    FRONT_left_motor_stop()
    BACK_left_motor_stop()
    FRONT_right_motor_forward()
    BACK_right_motor_forward()
    time.sleep(duration)
    stop_motors()

def turn_right(duration=1):
    FRONT_left_motor_forward()
    BACK_left_motor_forward()
    FRONT_right_motor_stop()
    BACK_right_motor_stop()
    time.sleep(duration)
    stop_motors()

# --- Flask Web Routes ---
@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>Car Control</title>
    </head>
    <body>
        <h1>Control Car</h1>
        <button onclick="location.href='/forward'">Forward</button>
        <button onclick="location.href='/backward'">Backward</button>
        <button onclick="location.href='/left'">Left</button>
        <button onclick="location.href='/right'">Right</button>
        <button onclick="location.href='/stop'">Stop</button>
    </body>
    </html>
    """

@app.route('/forward')
def forward():
    move_forward()
    return redirect(url_for('index'))

@app.route('/backward')
def backward():
    move_backward()
    return redirect(url_for('index'))

@app.route('/left')
def left():
    turn_left()
    return redirect(url_for('index'))

@app.route('/right')
def right():
    turn_right(0.5)
    return redirect(url_for('index'))

@app.route('/stop')
def stop():
    stop_motors()
    return redirect(url_for('index'))

def run_control_server():
    """
    Starts the Flask control server. In a finally block, ensures that GPIO pins are cleaned up.
    """
    try:
        app.run(host='0.0.0.0', debug=True, use_reloader=False)
    finally:
        GPIO.cleanup()

# ------------------ Main Program ------------------
if __name__ == '__main__':
    # Start the sensor loop in its own daemon thread
    
    tag_thread = threading.Thread(target=apriltag_loop, daemon=True)
    tag_thread.start()
    sensor_thread = threading.Thread(target=sensor_loop, daemon=True)
    sensor_thread.start()

    # Start the control server in its own daemon thread
    control_thread = threading.Thread(target=run_control_server, daemon=True)
    control_thread.start()

    # Main thread: periodically print the estimated sensor position
    try:
        while True:
            if calibrated:
                print("Forward {} Estimated Position (m): X={:.3f}, Y={:.3f}".format(currentlyForward,
                    estimated_position[0], estimated_position[1]
                ))
                
            time.sleep(0.05)  # Update display every second
    except KeyboardInterrupt:
        print("Exiting main program.")
