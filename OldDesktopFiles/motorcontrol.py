import RPi.GPIO as GPIO
import time

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
    """Run left front motor in the forward direction."""
    GPIO.output(FRONT_LEFT_PIN1, GPIO.HIGH)
    GPIO.output(FRONT_LEFT_PIN2, GPIO.LOW)

def FRONT_left_motor_backward():
    """Run left front motor in the backward direction."""
    GPIO.output(FRONT_LEFT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_LEFT_PIN2, GPIO.HIGH)

def FRONT_left_motor_stop():
    """Stop left front motor."""
    GPIO.output(FRONT_LEFT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_LEFT_PIN2, GPIO.LOW)

def FRONT_right_motor_forward():
    """Run right front motor in the forward direction."""
    GPIO.output(FRONT_RIGHT_PIN1, GPIO.HIGH)
    GPIO.output(FRONT_RIGHT_PIN2, GPIO.LOW)

def FRONT_right_motor_backward():
    """Run right front motor in the backward direction."""
    GPIO.output(FRONT_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_RIGHT_PIN2, GPIO.HIGH)

def FRONT_right_motor_stop():
    """Stop right front motor."""
    GPIO.output(FRONT_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(FRONT_RIGHT_PIN2, GPIO.LOW)

# ----- BACK Motor Functions -----
def BACK_left_motor_forward():
    """Run left back motor in the forward direction."""
    GPIO.output(BACK_LEFT_PIN1, GPIO.HIGH)
    GPIO.output(BACK_LEFT_PIN2, GPIO.LOW)

def BACK_left_motor_backward():
    """Run left back motor in the backward direction."""
    GPIO.output(BACK_LEFT_PIN1, GPIO.LOW)
    GPIO.output(BACK_LEFT_PIN2, GPIO.HIGH)

def BACK_left_motor_stop():
    """Stop left back motor."""
    GPIO.output(BACK_LEFT_PIN1, GPIO.LOW)
    GPIO.output(BACK_LEFT_PIN2, GPIO.LOW)

def BACK_right_motor_forward():
    """Run right back motor in the forward direction."""
    GPIO.output(BACK_RIGHT_PIN1, GPIO.HIGH)
    GPIO.output(BACK_RIGHT_PIN2, GPIO.LOW)

def BACK_right_motor_backward():
    """Run right back motor in the backward direction."""
    GPIO.output(BACK_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(BACK_RIGHT_PIN2, GPIO.HIGH)

def BACK_right_motor_stop():
    """Stop right back motor."""
    GPIO.output(BACK_RIGHT_PIN1, GPIO.LOW)
    GPIO.output(BACK_RIGHT_PIN2, GPIO.LOW)

# ----- Example Usage -----
try:
    # Test front motors
    print("Testing Front Motors:")
    print("Front left motor forward")
    FRONT_left_motor_forward()
    time.sleep(1)
    FRONT_left_motor_stop()
    time.sleep(0.5)

    print("Front left motor backward")
    FRONT_left_motor_backward()
    time.sleep(1)
    FRONT_left_motor_stop()
    time.sleep(0.5)

    print("Front right motor forward")
    FRONT_right_motor_forward()
    time.sleep(1)
    FRONT_right_motor_stop()
    time.sleep(0.5)

    print("Front right motor backward")
    FRONT_right_motor_backward()
    time.sleep(1)
    FRONT_right_motor_stop()
    time.sleep(0.5)

    # Test back motors
    print("\nTesting Back Motors:")
    print("Back left motor forward")
    BACK_left_motor_forward()
    time.sleep(1)
    BACK_left_motor_stop()
    time.sleep(0.5)

    print("Back left motor backward")
    BACK_left_motor_backward()
    time.sleep(1)
    BACK_left_motor_stop()
    time.sleep(0.5)

    print("Back right motor forward")
    BACK_right_motor_forward()
    time.sleep(1)
    BACK_right_motor_stop()
    time.sleep(0.5)

    print("Back right motor backward")
    BACK_right_motor_backward()
    time.sleep(1)
    BACK_right_motor_stop()

finally:
    # Clean up the GPIO settings when done
    GPIO.cleanup()
