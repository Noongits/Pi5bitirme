import time
import variables
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
    variables.currentlyForward = True
    time.sleep(duration)
    variables.currentlyForward = False
    stop_motors()

def move_backward(duration=1):
    FRONT_left_motor_backward()
    FRONT_right_motor_backward()
    BACK_left_motor_backward()
    BACK_right_motor_backward()
    variables.currentlyBackward = True
    time.sleep(duration)
    variables.currentlyBackward = False
    stop_motors()

def turn_left(duration=0.75):
    #FRONT_left_motor_stop()
    #BACK_left_motor_stop()
    BACK_left_motor_backward()
    FRONT_left_motor_backward()
    FRONT_right_motor_forward()
    BACK_right_motor_forward()
    variables.currentlyLeft = True
    time.sleep(duration)
    variables.currentlyLeft = False
    stop_motors()

import time
def turn_deg(degree=90, timeout=5.0, second_turn=False):
    # 1) record the true start and build a raw target
    start  = variables.current_direction      # e.g. –1 stays –1
    target = start + degree                   # e.g. –1 + 10 → 9
    tolerance = abs(degree) * 0.1             # 10% of the turn

    # 2) kick off the motors
    if degree > 0:
        # turn left
        BACK_left_motor_backward()
        FRONT_left_motor_backward()
        FRONT_right_motor_forward()
        BACK_right_motor_forward()
        variables.currentlyLeft = True
    else:
        # turn right
        BACK_left_motor_forward()
        FRONT_left_motor_forward()
        FRONT_right_motor_backward()
        BACK_right_motor_backward()
        variables.currentlyRight = True

    start_time = time.monotonic()
    error = target - variables.current_direction

    # 3) loop until within tolerance, overshoot, or timeout
    while True:
        current = variables.current_direction
        error   = target - current

        # a) are we close enough?
        if abs(error) <= tolerance:
            reason = "within tolerance"
            break

        # b) did we go past it?
        if degree > 0 and current >= target:
            reason = "overshoot detected"
            break
        if degree < 0 and current <= target:
            reason = "overshoot detected"
            break

        # c) timeout
        if time.monotonic() - start_time >= timeout:
            reason = "timeout"
            break

        time.sleep(0.01)

    # 4) stop and clear flags
    stop_motors()
    variables.currentlyLeft = variables.currentlyRight = False

    # 5) diagnostics
    print(f"Exit reason: {reason}")
    print(f"  start   : {start:.2f}°")
    print(f"  target  : {target:.2f}°")
    print(f"  current : {variables.current_direction:.2f}°  (error {error:.2f}°)")
    print(f"  tolerance: {tolerance:.2f}°, timeout: {timeout}s")

    time.sleep(1)
    # 6) small correction pass if still way off
    if abs(error) > 4 and not second_turn:
        print(f"DUZELTME GELDI°")
        correction = target - variables.current_direction
        print(f"Performing correction turn of {correction:.2f}°")
        turn_deg(correction, timeout, True)

  


def turn_right(duration=0.75):
    FRONT_left_motor_forward()
    BACK_left_motor_forward()
    #FRONT_right_motor_stop()
    #BACK_right_motor_stop()
    BACK_right_motor_backward()
    FRONT_right_motor_backward()
    variables.currentlyRight = True
    time.sleep(duration)
    variables.currentlyRight = False
    stop_motors()