import threading
from motor_controller import *
from web import run_control_server
from apriltag_detection import *
import state
import time


if __name__ == '__main__':
    control_thread = threading.Thread(target=run_control_server, daemon=True)
    control_thread.start()

    apriltag_thread = threading.Thread(target=detect_apriltag, daemon=True)
    apriltag_thread.start()

    try:
        while True:
            if state.calibrated and False:
                print("Forward {} Estimated Position (m): X={:.3f}, Y={:.3f}".format(
                    state.currentlyForward,
                    state.estimated_position[0], state.estimated_position[1]
                ))
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop_motors()
        print("Exiting main program.")
