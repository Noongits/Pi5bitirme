import numpy as np
from datetime import datetime
import variables
import time


def print_variable(name, value):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    if value is None:
        print(f"[{timestamp}] {name}: None")
    else:
        print(f"[{timestamp}] {name}: {value}")


def main():
    while True:
        print("\n--- Car Status Report ---\n")
        print_variable("Destination", variables.destination)
        print_variable("Estimated Position", variables.estimated_position)
        print_variable("Estimated Orientation (IMU)", variables.estimated_orientation)
        print_variable("Position Estimated by Wheel Sensors", variables.car_pose_tyresensor)
        print_variable("Distance to Eiffel", variables.eiffel_distance)
        print_variable("Eiffel Angle", variables.eiffel_angle)
        print_variable("Position Estimated by Eiffel Detection", variables.car_pose)
        print_variable("Detected AprilTags", variables.detected_tags)
        # print_variable("Position Estimated by AprilTag Detection", variables.tagarray)
        print("\n-------------------------\n")
        time.sleep(1)  # Delay for 1 second before the next update
