#!/usr/bin/env python3

import os
from datetime import datetime
from picamera2 import Picamera2
import time
import signal
import sys

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully."""
    print("\nStopping capture...")
    sys.exit(0)

def setup_camera(camera_num):
    """Initialize and configure a camera with specified settings."""
    try:
        camera = Picamera2(camera_num)
        config = camera.create_preview_configuration(
            main={"size": (1280, 960), "format": "RGB888"}
        )
        camera.configure(config)
        camera.start()
        return camera
    except Exception as e:
        print(f"Error setting up camera {camera_num}: {e}")
        return None

def capture_and_save_images(camera1, camera2, capture_count):
    """Capture images from both cameras and save them with timestamps."""
    try:
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Capture images
        image1 = camera1.capture_array()
        image2 = camera2.capture_array()

        # Rotate images 180 degrees (flip both horizontally and vertically)
        image1 = image1[::-1, ::-1]
        image2 = image2[::-1, ::-1]

        # Save images
        from PIL import Image
        Image.fromarray(image1).save(f"capture_images/camera1_{timestamp}.jpg")
        Image.fromarray(image2).save(f"capture_images/camera2_{timestamp}.jpg")
        
        print(f"Capture #{capture_count} - Images saved with timestamp: {timestamp}")

    except Exception as e:
        print(f"Error during capture: {e}")

def main():
    """Main function to continuously capture images until interrupted."""
    # Set up signal handler for graceful exit
    signal.signal(signal.SIGINT, signal_handler)

    # Create capture_images directory if it doesn't exist
    if not os.path.exists("capture_images"):
        os.makedirs("capture_images")

    # Initialize cameras
    print("Initializing cameras...")
    camera1 = setup_camera(0)  # First camera
    camera2 = setup_camera(1)  # Second camera

    if not camera1 or not camera2:
        print("Failed to initialize one or both cameras")
        return

    try:
        capture_count = 0
        print("Starting continuous capture. Press Ctrl+C to stop.")
        print("Capturing images every 1 second...")
        
        while True:
            capture_count += 1
            capture_and_save_images(camera1, camera2, capture_count)
            time.sleep(1)  # Wait for 1 second between captures

    except Exception as e:
        print(f"Unexpected error: {e}")
    
    finally:
        # Clean up
        print("Cleaning up cameras...")
        camera1.stop()
        camera2.stop()

if __name__ == "__main__":
    main() 