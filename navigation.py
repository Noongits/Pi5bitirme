from motor_controller import *
import time
import state

def navigate(x, y):
    







    '''
    if abs(x) >= 5 or abs(z) >= 5:
        if x > 0 and abs(x) >= 5:
            FRONT_left_motor_forward()
            FRONT_right_motor_forward()
            BACK_left_motor_forward()
            BACK_right_motor_forward()
            state.currentlyForward = True
            print("move forward")
            while True:
                target_x = x / 100.0  # Convert cm to meters
                if abs(state.estimated_position[0]) > 5:
                    break
                time.sleep(0.01)
            stop_motors()
            state.currentlyForward = False

        elif x < 0 and abs(x) >= 5:
            FRONT_left_motor_backward()
            FRONT_right_motor_backward()
            BACK_left_motor_backward()
            BACK_right_motor_backward()
            state.urrentlyBackward = True
            print("move backward")
            while True:
                if state.estimated_position[0] * 1000 == x:
                    break
                time.sleep(0.01)
            stop_motors()
            state.currentlyBackward = False
            
        if z > 0 and abs(z) >= 5:
            turn_right()
            FRONT_left_motor_forward()
            FRONT_right_motor_forward()
            BACK_left_motor_forward()
            BACK_right_motor_forward()
            state.currentlyForward = True
            print("turn right")
            while True:
                if state.estimated_position[1] * 1000 == z:
                    break
                time.sleep(0.01)
            stop_motors()
            state.currentlyForward = False
            
        elif z < 0 and abs(z) >= 5:
            turn_left()
            FRONT_left_motor_forward()
            FRONT_right_motor_forward()
            BACK_left_motor_forward()
            BACK_right_motor_forward()
            state.currentlyForward = True
            print("turn left")
            while True:
                if state.estimated_position[1] * 1000 == z:
                    break
                time.sleep(0.01)
            stop_motors()
            state.currentlyForward = False
            
    else:
        print("Already at destination")
    '''