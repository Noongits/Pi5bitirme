import variables
import detect_and_crop
import angle_classification
import stereo_distance
import numpy as np
import math
import time
import threading
from get_frames import *

def localise_with_eiffel():
    camera_thread = threading.Thread(target=capture_frames, daemon=True)
    camera_thread.start()

    while True:
        try:
            # Check if we have valid frames
            if variables.leftcam is None or variables.rightcam is None:
                print("Waiting for camera frames...")
                time.sleep(0.1)
                continue

            detect_and_crop.process_frames(variables.leftcam, variables.rightcam)
            
            # Only proceed with angle and distance if we have a valid crop
            if variables.cropped_eiffel is not None: # TODO: eiffel detection is not working
                angle_classification.main()
                stereo_distance.main()

                # Get the known Eiffel Tower location and measurements
                eiffel_loc = np.array(variables.eiffel_location)
                angle = variables.eiffel_angle
                distance = variables.eiffel_distance

                if angle is not None and distance is not None:
                    # Convert angle to radians
                    angle_rad = math.radians(angle)
                    
                    # Calculate the camera's position using triangulation
                    # The camera's position will be at a point that forms a right triangle
                    # with the Eiffel Tower, where:
                    # - The hypotenuse is the distance to the Eiffel Tower
                    # - The angle is the measured angle to the Eiffel Tower
                    # - We can calculate the x and y offsets using trigonometry
                    
                    # Calculate the offsets from the Eiffel Tower
                    x_offset = distance * math.sin(angle_rad)
                    y_offset = distance * math.cos(angle_rad)
                    
                    # Calculate the camera's position by subtracting the offsets from the Eiffel Tower location
                    camera_x = eiffel_loc[0] - x_offset
                    camera_y = eiffel_loc[1]  # Assuming same height as Eiffel Tower for now
                    camera_z = eiffel_loc[2] - y_offset
                    
                    # Update the car's pose with the calculated position
                    variables.car_pose = np.array([camera_x, camera_z, angle])
                    
                    print(f"Vehicle position calculated: x={camera_x:.2f}, y={camera_z:.2f}, angle={angle:.2f}")
                else:
                    print("Warning: Could not localize - missing angle or distance measurements")
            else:
                print("Waiting for Eiffel Tower detection...")
                time.sleep(0.1)

        except Exception as e:
            print(f"Error in localise_with_eiffel(): {str(e)}")
            time.sleep(0.1)

if __name__ == "__main__":
    localise_with_eiffel()
