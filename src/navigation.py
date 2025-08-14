import manhattan_navigation
from motor_controller import *
from apriltag_detection import *
from localisation import *
import simulate_road_network
import variables
from localisation_eiffel import *
import threading

TOL = 0.2

def measure_angle(angle):
        # Wrap to (-180, 180]
        angle = ((angle + 180) % 360) - 180
        # Convert -180 to 180 to keep range [-179, 180]
        return 180 if angle == -180 else angle

def navigate():
    time.sleep(2)
   

    apriltag_thread = threading.Thread(target=detect_apriltag, daemon=True)
    apriltag_thread.start()

    eiffel_thread = threading.Thread(target=localise_with_eiffel, daemon=True)
    eiffel_thread.start()

    localisation_thread = threading.Thread(target=self_localise, daemon=True)
    localisation_thread.start()

    variables.destination = np.array([
        float(input("Enter destination X (meters): ")),
        float(input("Enter destination Z (meters): ")),
    ])

    if variables.donotmove:
        return

    if variables.nav_mode == 0 : # Straight line
        return # Have to turn a certain angle

    elif variables.nav_mode == 1: # Manhattan Navigation
        variables.road_network = simulate_road_network.create_road_network()
        closest_point = manhattan_navigation.find_closest_road_point(variables.destination[0], variables.destination[1])
        #path = manhattan_navigation.find_path((0.0, 0.0), closest_point)
        # print(f"Destination: {variables.destination}")
        # print(f"Closest point on road: {closest_point}")
        if variables.current_direction < -45:

            print("on axis x")
            if closest_point[0] > 0:
                turn_right()
                #variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[0] < 0:
                turn_left()
                #variables.current_direction = measure_angle(variables.current_direction - 90)
            while abs(closest_point[0] - variables.car_pose[0]) > TOL:
                #print(variables.car_pose)
                #print(closest_point)
                #print("get closer")
                variables.canstop = True
                move_forward()
                time.sleep(0.01)

            #print("close enough")
            if variables.canstop == True:
                stop_motors()
                variables.canstop = False

            if closest_point[1] > 0 and variables.current_direction == 90: # +y / +90
                turn_left()
                #variables.current_direction = measure_angle(variables.current_direction - 90)
            elif closest_point[1] < 0 and variables.current_direction == 90: # -y / +90
                turn_right()
                #variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[1] > 0 and variables.current_direction == -90: # +y / -90
                turn_right()
                #variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[1] < 0 and variables.current_direction == -90: # -y / -90
                turn_left()
                #variables.current_direction = measure_angle(variables.current_direction - 90)
            while abs(variables.car_pose[0]) < abs(closest_point[0]):
                move_forward()

        else:
            
            print("on axis z")
            if closest_point[1] < 0:
                turn_right()
                turn_right()
                variables.current_direction = measure_angle(variables.current_direction + 180)
            while abs(closest_point[1] - variables.car_pose[2]) > TOL:
                #print(variables.car_pose)
                #print(closest_point)
                #print("MOVE_FORWARD")
                variables.canstop = True
                move_forward()
                time.sleep(0.01)
           # print("close enough")

            
            if variables.canstop == True:
                #stop_motors()
                #print("stop motors Z")
                variables.canstop = False

            if closest_point[0] < 0 and abs(variables.current_direction) < 15: # +x / 0
                
                turn_deg(90)
                #variables.current_direction = measure_angle(variables.current_direction - 90)
            elif closest_point[0] > 0 and  abs(variables.current_direction) < 15: # -x / 0
                
                turn_deg(-90)
                print("saga dondu")
                #variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[0] < 0 and  variables.current_direction > 175 and variables.current_direction < 185: # +x / 180
                turn_deg(-90)
                #variables.current_direction = measure_angle(variables.current_direction + 90)
            elif closest_point[0] > 0 and variables.current_direction > 175 and variables.current_direction < 185: # -x / 180
                #print("turn left 2")
                turn_deg(90)
                #variables.current_direction = measure_angle(variables.current_direction - 90)
            print("burada" + str(variables.destination[0]) + " " + str(variables.car_pose[0]))
            while  abs(variables.destination[0]) > abs(variables.car_pose[0]):
                print("buraya girdi")
                move_forward()
            stop_motors()
        
    
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


