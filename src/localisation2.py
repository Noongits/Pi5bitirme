import variables
import time
import numpy as np
import threading

def get_position_estimate():
    """
    Returns the most reliable position estimate based on the following hierarchy:
    1. AprilTag localization (highest priority)
    2. Eiffel Tower localization
    3. Sensor-based localization (base estimate)
    """
    # Start with sensor-based position as the base estimate
    position = variables.car_pose_tyresensor.copy()
    
    # If Eiffel Tower localization is available, use it
    if (variables.eiffel_angle is not None and 
        variables.eiffel_distance is not None and 
        variables.cropped_eiffel is not None):
        # Use the position from Eiffel Tower localization
        position = variables.car_pose.copy()
    
    # If AprilTag localization is available, it takes highest priority
    if variables.detected_tags:
        # Calculate position from AprilTags
        tag_positions = []
        for tag in variables.detected_tags:
            tag_relative = variables.tagarray[tag]  # Tag wrt camera
            tag_world = variables.APRILTAG_COORDS[tag]  # Tag real position
            car_position = np.array([
                tag_world[0] - tag_relative[0],
                tag_world[1] + tag_relative[1],
                tag_world[2] - tag_relative[2]
            ])
            tag_positions.append(car_position)
        
        if tag_positions:
            # Average the positions from all detected tags
            position = np.mean(tag_positions, axis=0)
    
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
            print(f"Current position estimate: {current_position}")
            if variables.detected_tags:
                print(f"Using AprilTag localization with tags: {variables.detected_tags}")
            elif variables.eiffel_angle is not None and variables.eiffel_distance is not None:
                print("Using Eiffel Tower localization")
            else:
                print("Using sensor-based localization")
            
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage
            
        except Exception as e:
            print(f"Error in self_localise(): {str(e)}")
            time.sleep(0.1)

if __name__ == "__main__":
    self_localise()
