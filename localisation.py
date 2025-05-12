from variables import *
from apriltag_detection import *

def self_localise():
    # Reset car_pose for this iteration
    variables.car_pose = np.array([0.0, 0.0, 0.0])
    print(f"Estimated position: {variables.car_pose}")
    
    for tag in detected_tags:
        relative_pos = tagarray[tag] # Tag wrt camera
        tag_world = APRILTAG_COORDS[tag] # Tag real position
        car_position_est = tag_world - relative_pos  # Car position in world, calculated according to tag detected
        variables.car_pose += car_position_est
        print(f"Tag World: {tag_world} Tag reletive: {relative_pos}")
    variables.car_pose /= len(detected_tags)
    
