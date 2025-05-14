from motor_controller import *
import time

def test(duration):
    FRONT_left_motor_forward()
    FRONT_right_motor_forward()
    BACK_left_motor_forward()
    BACK_right_motor_forward()
    time.sleep(duration)
    stop_motors()

test(1)