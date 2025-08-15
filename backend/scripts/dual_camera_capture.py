#!/usr/bin/env python3

import os
from datetime import datetime
from picamera2 import Picamera2
import time
import cv2

def setup_camera(camera_num):
    """Initialize and configure a camera with specified settings."""
    camera = Picamera2(camera_num)
    config = camera.create_preview_configuration(
        main={"size": (1280, 960), "format": "BGR888"}
    )
    camera.configure(config)
    camera.start()
    time.sleep(2)  # Allow camera to warm up
    return camera

def create_output_folder():
    """Create a timestamped folder for saving images."""
    folder_name = f"captured_images"
    os.makedirs(folder_name, exist_ok=True)
    return folder_name

def capture_and_save_images(camera1, camera2, output_folder):
    """Capture images from both cameras, flip them, and save to disk."""
    # Capture images
    image1 = camera1.capture_array()
    image2 = camera2.capture_array()
    
    # Flip both images vertically and horizontally
    image1_flipped = cv2.flip(image1, -1)  # -1 flips both vertically and horizontally
    image2_flipped = cv2.flip(image2, -1)
    
    # Generate filenames with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename1 = os.path.join(output_folder, f"rightcam_{timestamp}.jpg")
    filename2 = os.path.join(output_folder, f"leftcam_{timestamp}.jpg")
    
    # Save images
    cv2.imwrite(filename1, cv2.cvtColor(image1_flipped, cv2.COLOR_RGB2BGR))
    cv2.imwrite(filename2, cv2.cvtColor(image2_flipped, cv2.COLOR_RGB2BGR))
    
    print(f"Images saved: {filename1} and {filename2}")

def main():
    try:
        # Initialize cameras
        print("Initializing cameras...")
        camera1 = setup_camera(0)  # First camera
        camera2 = setup_camera(1)  # Second camera
        
        # Create output folder
        output_folder = create_output_folder()
        print(f"Created output folder: {output_folder}")
        
        # Capture and save images
        print("Capturing images...")
        capture_and_save_images(camera1, camera2, output_folder)
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Clean up
        try:
            camera1.stop()
            camera2.stop()
        except:
            pass

if __name__ == "__main__":
    main() 