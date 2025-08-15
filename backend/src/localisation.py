import variables
import time
import numpy as np
import math

def get_position_estimate():
    """
    Returns the most reliable position estimate based on the following hierarchy:
    1. AprilTag localization (highest priority)
    2. Eiffel Tower localization
    3. Sensor-based localization (base estimate)
    """
    # Start with sensor-based position as the base estimate
    position = variables.car_pose_tyresensor_fixed
    
    # If Eiffel Tower localization is available, use it
    if (variables.eiffel_angle is not None and 
        variables.eiffel_distance is not None and 
        variables.cropped_eiffel is not None):
        # Use the position from Eiffel Tower localization
        position = variables.car_pose
    
    # If AprilTag localization is available, it takes highest priority
    if variables.detected_tags:
        # Calculate position from AprilTags
        tag_positions = []
        yaw = variables.current_direction * math.pi / 180  # in radians
        cy, sy = math.cos(yaw), math.sin(yaw)
        for tag in variables.detected_tags:
                
            tag_rel   = np.array(variables.tagarray[tag])       # [x_cam, y_cam, z_cam]
            tag_world = np.array(variables.APRILTAG_COORDS[tag])# [x_w,   y_w,   z_w  ]

            # rotate the cam→tag vector into world coords (only X/Z):
            x_rel_w =  cy * tag_rel[0] - sy * tag_rel[2]
            z_rel_w =  sy * tag_rel[0] + cy * tag_rel[2]

            # now camera = tag_world − [x_rel_w, y_rel, z_rel_w]
            car_pos = np.array([
                tag_world[0] - x_rel_w,
                tag_world[1] + tag_rel[1],  # keep your original Y‐axis handling
                tag_world[2] - z_rel_w
            ])
            tag_positions.append(car_pos)

            
        
        if tag_positions:
            # Average the positions from all detected tags
            position = np.mean(tag_positions, axis=0)
    
    if variables.detected_tags:
        variables.car_pose_tyresensor_fixed = position
        print(f"Car pose fixed from tyre sensor: {variables.car_pose_tyresensor} tag position: {position}")
    return position

def self_localise():
    """
    Main localization loop that continuously updates the car's position
    using the hierarchical localization system.
    """
    while True:
        try:
            # Get the most reliable position estimate
            current_position = get_position_estimate()
            
            # Update the car's pose with the most reliable estimate
            variables.car_pose = current_position
            
            # Print debug information
            #print(f"Current position estimate: {current_position[0:2]}")
            #if variables.detected_tags:
                #print(f"Using AprilTag localization with tags: {variables.detected_tags}")
            #elif variables.eiffel_angle is not None and variables.eiffel_distance is not None:
                #print("Using Eiffel Tower localization")
            #else:
                #print("Using sensor-based localization")
            
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage
            
        except Exception as e:
            print(f"Error in self_localise(): {str(e)}")
            time.sleep(0.1)

if __name__ == "__main__":
    self_localise()
