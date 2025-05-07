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

def sensor_loop():
    """
    Initializes the sensor, performs calibration, and continuously reads sensor data
    to update the estimated_position variable.
    """
    # Initialize sensor and perform calibration
    mpu6050.mpu6050_init()
    ax_off, ay_off, az_off, gx_off, gy_off, gz_off = mpu6050.calibrate(num_samples=3000)
    global calibrated
    calibrated = True

    move_thread = threading.Thread(target=goToDestination, args=(50, 100), daemon=True)
    #move_thread.start()

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
            #print(ax, ay, az)

            if not currentlyForward and not currentlyBackward:
                velocity[0] = 0.0
                velocity[1] = 0.0

            # Integrate acceleration to update velocity (with damping)
            if True:
                velocity[0] = (velocity[0] + ax * dt) * damping_factor
                #ax = 0
                velocity[1] = (velocity[1] + ay * dt) * damping_factor
                #ay = 0
                velocity[2] = (velocity[2] + az * dt) * damping_factor
                #az = 0
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
    currentlyForward = False
    stop_motors()

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



def goToDestination(x, z):
    if abs(x) >= 25 or abs(z) >= 25:
        if x > 0 and abs(x) >= 25:
            FRONT_left_motor_forward()
            FRONT_right_motor_forward()
            BACK_left_motor_forward()
            BACK_right_motor_forward()
            currentlyForward = True
            print("move forward")
            while True:
                target_x = x / 100.0  # Convert cm to meters
                if abs(estimated_position[0] - target_x) < 0.01:  # 1 cm tolerance
                    break
                time.sleep(0.01)
            stop_motors()
            currentlyForward = False

        elif x < 0 and abs(x) >= 25:
            FRONT_left_motor_backward()
            FRONT_right_motor_backward()
            BACK_left_motor_backward()
            BACK_right_motor_backward()
            currentlyBackward = True
            print("move backward")
            while True:
                if estimated_position[0] * 1000 == x:
                    break
                time.sleep(0.01)
            stop_motors()
            currentlyBackward = False
            
        if z > 0 and abs(z) >= 25:
            turn_right()
            FRONT_left_motor_forward()
            FRONT_right_motor_forward()
            BACK_left_motor_forward()
            BACK_right_motor_forward()
            currentlyForward = True
            print("turn right")
            while True:
                if estimated_position[1] * 1000 == z:
                    break
                time.sleep(0.01)
            stop_motors()
            currentlyForward = False
            
        elif z < 0 and abs(z) >= 25:
            turn_left()
            FRONT_left_motor_forward()
            FRONT_right_motor_forward()
            BACK_left_motor_forward()
            BACK_right_motor_forward()
            currentlyForward = True
            print("turn left")
            while True:
                if estimated_position[1] * 1000 == z:
                    break
                time.sleep(0.01)
            stop_motors()
            currentlyForward = False
            
    else:
        print("Already at destination")



# ------------------ Main Program ------------------
if __name__ == '__main__':
    # Start the sensor loop in its own daemon thread
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
        stop_motors()
        print("Exiting main program.")
