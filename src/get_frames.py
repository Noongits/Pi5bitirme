from picamera2 import Picamera2
import variables
import signal
import sys

picam1 = Picamera2(camera_num=0)
picam2 = Picamera2(camera_num=1)
preview_config1 = picam1.create_preview_configuration(main={"size": (1280, 960)})
preview_config2 = picam2.create_preview_configuration(main={"size": (1280, 960)})
picam1.configure(preview_config1)
picam2.configure(preview_config2)
picam1.set_controls({"FrameRate": 30})
picam2.set_controls({"FrameRate": 30})
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

def signal_handler(sig, frame):
    print("Exiting...")
    picam1.stop()
    picam2.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
