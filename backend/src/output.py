import numpy as np
from datetime import datetime
import variables
import time
import os


def print_variable(name, value):
    if value is None:
        return f"{name}: None"
    else:
        return f"{name}: {value}"

def format_status_report():
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    report = f"\n--- Car Status Report --- [{timestamp}] ---\n"
    report += print_variable("Destination", variables.destination) + "\n"
    report += print_variable("Estimated Position", variables.car_pose) + "\n"
    report += print_variable("Estimated Orientation (IMU)", np.round(variables.estimated_orientation, 3)) + "\n"
    report += print_variable("Current Direction", np.round(variables.current_direction, 2)) + "\n"
    report += print_variable("Position Estimated by Wheel Sensors", variables.car_pose_tyresensor) + "\n"
    report += print_variable("Distance to Eiffel", variables.eiffel_distance) + "\n"
    report += print_variable("Eiffel Angle", variables.eiffel_angle) + "\n"
    report += print_variable("Position Estimated by Eiffel Detection", variables.car_pose) + "\n"
    report += print_variable("Detected AprilTags", variables.detected_tags) + "\n"
    report += "\n-------------------------\n"
    return report

def main():
    iteration_count = 0
    save_to_file = False
    
    if save_to_file:
        # Create log file name with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = f"new_1_2cam_2tag_ARmodel_{timestamp}.txt"
    
        # Create or clear the log file
        with open(log_file, 'w') as f:
            f.write("Car Status Log - First 50 Iterations\n")
            f.write("=" * 40 + "\n\n")
        time.sleep(7)
    while True:
        if variables.destination is not None:
            report = format_status_report()
            print(report)  # Always print to console
            
            # Save to file for first 50 iterations
            if iteration_count < 100 and save_to_file:
                with open(log_file, 'a') as f:
                    f.write(report)
                iteration_count += 1
            
            time.sleep(0.01)  # Delay for 1 second before the next update
            
        if variables.destination_reached:
            print("Destination reached")
            break

if __name__ == "__main__":
    main()
