import numpy as np
from variables import (destination, estimated_position, estimated_orientation, 
                      car_pose_tyresensor, eiffel_distance, eiffel_angle, 
                      car_pose, detected_tags, tagarray)


def print_variable(name, value):
    if value is None:
        print(f"{name}: None")
    else:
        print(f"{name}: {value}")


def main():
    print("\n--- Car Status Report ---\n")
    print_variable("Destination", destination)
    print_variable("Estimated Position", estimated_position)
    print_variable("Estimated Orientation (IMU)", estimated_orientation)
    print_variable("Position Estimated by Wheel Sensors", car_pose_tyresensor)
    print_variable("Distance to Eiffel", eiffel_distance)
    print_variable("Eiffel Angle", eiffel_angle)
    print_variable("Position Estimated by Eiffel Detection", car_pose)
    print_variable("Detected AprilTags", detected_tags)
    print_variable("Position Estimated by AprilTag Detection", tagarray)
    print("\n-------------------------\n")
