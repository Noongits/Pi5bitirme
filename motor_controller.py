import time
import state
import RPi.GPIO as GPIO

currentlyForward = False
currentlyBackward = False
currentlyRight = False
currentlyLeft = False

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
   
    FRONT_left_motor_forward()
    FRONT_right_motor_forward()
    BACK_left_motor_forward()
    BACK_right_motor_forward()
    state.currentlyForward = True
    time.sleep(duration)
    state.currentlyForward = False
    stop_motors()

def move_backward(duration=1):
    FRONT_left_motor_backward()
    FRONT_right_motor_backward()
    BACK_left_motor_backward()
    BACK_right_motor_backward()
    state.currentlyBackward = True
    time.sleep(duration)
    state.currentlyBackward = False
    stop_motors()

def turn_left(duration=1.5):
    FRONT_left_motor_stop()
    BACK_left_motor_stop()
    FRONT_right_motor_forward()
    BACK_right_motor_forward()
    state.currentlyLeft = True
    time.sleep(duration)
    state.currentlyLeft = False
    stop_motors()

def turn_right(duration=1.5):
    FRONT_left_motor_forward()
    BACK_left_motor_forward()
    FRONT_right_motor_stop()
    BACK_right_motor_stop()
    state.currentlyRight = True
    time.sleep(duration)
    state.currentlyRight = False
    stop_motors()