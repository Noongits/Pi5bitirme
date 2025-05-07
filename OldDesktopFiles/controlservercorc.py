from flask import Flask, redirect, url_for
from flask_cors import CORS  # Import Flask-CORS
import RPi.GPIO as GPIO
import time

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# --- GPIO and Motor Setup ---
# Define the GPIO pins using BCM numbering
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
def move_forward(duration=1):
    FRONT_left_motor_forward()
    FRONT_right_motor_forward()
    BACK_left_motor_forward()
    BACK_right_motor_forward()
    time.sleep(duration)
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

def stop_motors():
    FRONT_left_motor_stop()
    FRONT_right_motor_stop()
    BACK_left_motor_stop()
    BACK_right_motor_stop()

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

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', debug=True, use_reloader=False)
    finally:
        GPIO.cleanup()
