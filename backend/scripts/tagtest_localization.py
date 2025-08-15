import cv2
import apriltag
import numpy as np
import time
from picamera2 import Picamera2
from datetime import datetime
import csv
import os

# Test distance in meters - change this before each test run
TEST_DISTANCE = "2"  # meters

# Camera intrinsics (using values from existing code)
fx = 1270
fy = 1270
cx = 1280 / 2
cy = 960 / 2
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))

fx2 = 1057
fy2 = 1057
cx2 = 1280 / 2
cy2 = 960 / 2
camera_matrix2 = np.array([[fx2, 0, cx2], [0, fy2, cy2], [0, 0, 1]], dtype=np.float32)
dist_coeffs2 = np.zeros((5, 1))

# Tag size in meters
tag_size = 0.08
half_size = tag_size / 2.0
object_points = np.array([
    [-half_size,  half_size, 0],
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

# Known tag positions in meters (x, y, z)
TAG_POSITIONS = {
    0: np.array([0.0, 0.0, 0.0]),
    2: np.array([0.08, 0.0, 2.0]),
    3: np.array([-0.30, 0.0, 2.0]),
    6: np.array([0.0, 0.0, 2.0]),
    8: np.array([0.0, 0.0, 4.0])
}

timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

def setup_cameras():
    # Initialize cameras
    right_cam = Picamera2(0)
    left_cam = Picamera2(1)
    
    # Configure cameras
    config_left = left_cam.create_preview_configuration(main={"size": (1280, 960)})
    config_right = right_cam.create_preview_configuration(main={"size": (1280, 960)})
    
    left_cam.configure(config_left)
    right_cam.configure(config_right)
    
    # Start cameras
    left_cam.start()
    right_cam.start()
    
    return left_cam, right_cam

def detect_tags_in_frame(frame, camera_matrix, dist_coeffs):
    # Flip frame both horizontally and vertically
    frame = cv2.flip(frame, -1)
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect AprilTags
    detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))
    results = detector.detect(gray)
    
    positions = []
    for r in results:
        image_points = np.array(r.corners, dtype=np.float32)
        retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        if retval and r.tag_id in TAG_POSITIONS:
            tag_world = TAG_POSITIONS[r.tag_id]
            tag_relative = tvec.ravel()
            # Calculate car position relative to tag
            car_position = np.array([
                tag_world[0] - tag_relative[0],
                tag_world[1] + tag_relative[1],
                tag_world[2] - tag_relative[2]
            ])
            positions.append((r.tag_id, car_position))
            print(f"Tag {r.tag_id} detected at relative position: x={tag_relative[0]:.3f}m, y={tag_relative[1]:.3f}m, z={tag_relative[2]:.3f}m")
    
    return positions

def save_results_to_file(run_number, test_case, positions, avg_position):
    # Create results directory if it doesn't exist
    if not os.path.exists('test_results'):
        os.makedirs('test_results')
    
    filename = f'test_results/test_{TEST_DISTANCE}m_{timestamp}.csv'
    
    # Check if file exists to determine if we need to write headers
    file_exists = os.path.isfile(filename)
    
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        
        # Write headers if file is new
        if not file_exists:
            writer.writerow(['Run', 'Test Case', 'Test Distance (m)', 
                           'Car Position Relative to Tags', 'Average Car Position'])
        
        # Format individual tag positions
        tag_positions = []
        for tag_id, pos in positions:
            tag_positions.append(f"Tag {tag_id}: (x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f})")
        
        # Write the data
        writer.writerow([
            run_number,
            test_case,
            TEST_DISTANCE,
            ' | '.join(tag_positions) if tag_positions else "No tags detected",
            f"(x={avg_position[0]:.3f}, y={avg_position[1]:.3f}, z={avg_position[2]:.3f})" if avg_position is not None else "N/A"
        ])

def main():
    
    print("Initializing cameras...")
    left_cam, right_cam = setup_cameras()
    time.sleep(2)  # Give cameras time to warm up
    
    print(f"\nStarting test cases at {TEST_DISTANCE} meters...")
    print("Press Ctrl+C to exit")
    
    try:
        for run_number in range(1, 51):  # Changed to 1-based indexing
            # Get frames from both cameras
            left_frame = left_cam.capture_array()
            right_frame = right_cam.capture_array()
            
            # Test Case 1: Right camera only
            print(f"\nRun {run_number} - Test Case 1: Right Camera Only")
            right_positions = detect_tags_in_frame(right_frame, camera_matrix, dist_coeffs)
            if right_positions:
                avg_position = np.mean([pos for _, pos in right_positions], axis=0)
                print(f"Car's Relative Position: x={avg_position[0]:.3f}m, y={avg_position[1]:.3f}m, z={avg_position[2]:.3f}m")
                print(f"Detected tags: {[tag_id for tag_id, _ in right_positions]}")
                save_results_to_file(run_number, "Right Camera Only", right_positions, avg_position)
            else:
                print("No tags detected in right camera")
                save_results_to_file(run_number, "Right Camera Only", [], None)
            
            # Test Case 2: Both cameras
            print(f"\nRun {run_number} - Test Case 2: Both Cameras")
            left_positions = detect_tags_in_frame(left_frame, camera_matrix2, dist_coeffs2)
            all_positions = right_positions + left_positions
            
            if all_positions:
                avg_position = np.mean([pos for _, pos in all_positions], axis=0)
                print(f"Car's Relative Position: x={avg_position[0]:.3f}m, y={avg_position[1]:.3f}m, z={avg_position[2]:.3f}m")
                print(f"Detected tags: {[tag_id for tag_id, _ in all_positions]}")
                save_results_to_file(run_number, "Both Cameras", all_positions, avg_position)
            else:
                print("No tags detected in either camera")
                save_results_to_file(run_number, "Both Cameras", [], None)
            
            time.sleep(0.1)  # Small delay to prevent overwhelming the system
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        left_cam.stop()
        right_cam.stop()

if __name__ == "__main__":
    main()
