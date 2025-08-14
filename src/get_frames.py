import os
os.environ['LIBCAMERA_LOG_LEVELS'] = 'ERROR'

from picamera2 import Picamera2
import variables
import signal
import sys
import libcamera
import cv2
import numpy as np

from picamera2 import Picamera2

# Initialize both cameras
picam1 = Picamera2(camera_num=0)
picam2 = Picamera2(camera_num=1)

# Create preview configurations
preview_config1 = picam1.create_preview_configuration(main={"format": "BGR888", "size": (1280, 960)}, transform=libcamera.Transform(hflip=1, vflip=1))
preview_config2 = picam2.create_preview_configuration(main={"format": "BGR888", "size": (1280, 960)}, transform=libcamera.Transform(hflip=1, vflip=1))

# Apply configurations
picam1.configure(preview_config1)
picam2.configure(preview_config2)

# Set frame rate
picam1.set_controls({"FrameRate": 30})
picam2.set_controls({"FrameRate": 30})

# Start the cameras
picam1.start()
picam2.start()


def capture_frames():
    while True:
        variables.left_frame_imm = picam1.capture_array()
        if variables.leftlock.acquire(blocking=False):
            variables.leftcam = variables.left_frame_imm
            variables.leftlock.release()

        variables.right_frame_imm = picam2.capture_array()
        if variables.rightlock.acquire(blocking=False):
            variables.rightcam = variables.right_frame_imm
            variables.rightlock.release()
            
        # # Display frames
        # cv2.imshow('Left Camera', variables.left_frame_imm)
        # cv2.imshow('Right Camera', variables.right_frame_imm)
        
        # # Break loop if 'q' is pressed
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

def signal_handler(sig, frame):
    print("Exiting...")
    cv2.destroyAllWindows()  # Close all OpenCV windows
    picam1.stop()
    picam2.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
