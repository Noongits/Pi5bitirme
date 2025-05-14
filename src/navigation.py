import manhattan_navigation
from motor_controller import *
from apriltag_detection import *
from localisation import *
import simulate_road_network
import variables
import threading

def measure_angle(angle):
        # Wrap to (-180, 180]
        angle = ((angle + 180) % 360) - 180
        # Convert -180 to 180 to keep range [-179, 180]
        return 180 if angle == -180 else angle

def navigate():
    # Destination input
    variables.destination = np.array([
        float(input("Enter destination X (meters): ")),
        float(input("Enter destination Z (meters): ")),
        0.0
    ])

    apriltag_thread = threading.Thread(target=detect_apriltag, daemon=True)
    apriltag_thread.start()

    localisation_thread = threading.Thread(target=self_localise, daemon=True)
    localisation_thread.start()

    if variables.nav_mode == 0: # Straight line
        return # Have to turn a certain angle

    elif variables.nav_mode == 1: # Manhattan Navigation
        variables.road_network = simulate_road_network.create_road_network()
        closest_point = manhattan_navigation.find_closest_road_point(variables.destination[0], variables.destination[1])
        #path = manhattan_navigation.find_path((0.0, 0.0), closest_point)
        # print(f"Destination: {variables.destination}")
        # print(f"Closest point on road: {closest_point}")
        if variables.start_on_x:
            if closest_point[0] > 0:
                turn_right()
                variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[0] < 0:
                turn_left()
                variables.current_direction = measure_angle(variables.current_direction - 90)
            while abs(variables.car_pose[0]) < abs(closest_point[0]):
                move_forward()

            if closest_point[1] > 0 and variables.current_direction == 90: # +y / +90
                turn_left()
                variables.current_direction = measure_angle(variables.current_direction - 90)
            elif closest_point[1] < 0 and variables.current_direction == 90: # -y / +90
                turn_right()
                variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[1] > 0 and variables.current_direction == -90: # +y / -90
                turn_right()
                variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[1] < 0 and variables.current_direction == -90: # -y / -90
                turn_left()
                variables.current_direction = measure_angle(variables.current_direction - 90)
            while abs(variables.car_pose[1]) < abs(closest_point[1]):
                move_forward()

        else:
            if closest_point[1] < 0:
                turn_right()
                turn_right()
                variables.current_direction = measure_angle(variables.current_direction + 180)
            while abs(variables.car_pose[1]) < abs(closest_point[1]):
                move_forward()

            if closest_point[0] < 0 and variables.current_direction == 0: # +x / 0
                turn_left()
                variables.current_direction = measure_angle(variables.current_direction - 90)
            elif closest_point[0] > 0 and variables.current_direction == 0: # -x / 0
                turn_right()
                variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[0] < 0 and variables.current_direction == 180: # +x / 180
                turn_right()
                variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[0] > 0 and variables.current_direction == 180: # -x / 180
                turn_left()
                variables.current_direction = measure_angle(variables.current_direction - 90)
            while abs(variables.car_pose[0]) < abs(closest_point[0]):
                move_forward()
    
    elif variables.nav_mode == 2: # Move to AprilTag
        # TODO: Add function to turn to the correct angle
        if variables.tagarray[0, 2] > 0.5 and variables.currentlyForward == False:
            move_forward()
        if variables.tagarray[0, 2] <= 0.5 and variables.currentlyForward:
            stop_motors
    
    elif variables.nav_mode == 3: # Move to landmark
        return
    
    else:
        print("Invalid road number")
