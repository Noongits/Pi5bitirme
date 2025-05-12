from motor_controller import *
from apriltag_detection import *
from localisation import *
import time
import variables
import threading

def navigate():
    stage = 0
    '''
    # Destination input
    destination = np.array([
        float(input("Enter destination X (meters): ")),
        float(input("Enter destination Z (meters): ")),
        0.0
    ])
    '''
    
    apriltag_thread = threading.Thread(target=detect_apriltag, daemon=True)
    apriltag_thread.start()

    localisation_thread = threading.Thread(target=self_localise, daemon=True)
    localisation_thread.start()

    # STAGE 0: Move forward
    if stage == 0 and NavMesh:
        #distance = np.linalg.norm(relative_pos)
        #print(f"To tag {nearest_tag}: {distance:.2f} m")
        if variables.car_pose[2] < variables.destination[2]:
            move_forward()
            variables.currentlyForward = True
        else:
            stop_motors()
            variables.currentlyForward = False
            print("Close to tag. Preparing to turn...")
            stage = 1

    # STAGE 1: Turn left or right
    elif stage == 1 and NavMesh:
        dx = variables.destination[0] - variables.car_pose[0]
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
        dist_to_dest = np.linalg.norm(variables.destination[:2] - variables.car_pose[:2])
        print(f"Distance to destination: {dist_to_dest:.2f} m")
        if dist_to_dest > 0.2:
            move_forward()
            variables.currentlyForward = True
        else:
            stop_motors()
            variables.destination_reached = True
            print("Destination reached.")



    '''
    nearest_tag = min(detected_tags, key=lambda tid: np.linalg.norm(tagarray[tid]))
    relative_pos = tagarray[nearest_tag] # Tag wrt camera
    tag_world = APRILTAG_COORDS[nearest_tag] # Tag real position
    car_position_est = tag_world - relative_pos # Car position in world, calculated according to tag detected
    car_pose = car_position_est
    '''

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
