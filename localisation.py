import variables
import time
import numpy as np

def self_localise():
    while True:
        # Reset car_pose for this iteration
        print(f"Estimated position: {variables.car_pose}")
        print(f"Detected tags: {variables.detected_tags}")
        time.sleep(0.1)
        if variables.detected_tags:            
            for tag in variables.detected_tags:
                tag_relative = variables.tagarray[tag] # Tag wrt camera
                tag_world = variables.APRILTAG_COORDS[tag] # Tag real position
                car_position_est = np.array([tag_world[0] - tag_relative[0], tag_world[1] + tag_relative[1], tag_world[2] - tag_relative[2]])  # Car position in world, calculated according to tag detected
                variables.car_pose = car_position_est
                print(f"Tag World: {tag_world} Tag reletive: {tag_relative}")
            if len(variables.detected_tags) > 0:
                variables.car_pose /= len(variables.detected_tags)
