import threading
from motor_controller import *
from web import run_control_server
from apriltag_detection import *
from localisation import *
from navigation import *
import variables
import time

if __name__ == '__main__':
    control_thread = threading.Thread(target=run_control_server, daemon=True)
    control_thread.start()

    apriltag_thread = threading.Thread(target=detect_apriltag, daemon=True)
    apriltag_thread.start()

    localisation_thread = threading.Thread(target=self_localise, daemon=True)
    localisation_thread.start()

    navigation_thread = threading.Thread(target=navigate, daemon=True)
    navigation_thread.start()

    try:
        while True:
            if variables.calibrated and False:
                print("Forward {} Estimated Position (m): X={:.3f}, Y={:.3f}".format(
                    variables.currentlyForward,
                    variables.estimated_position[0], variables.estimated_position[1]
                ))
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop_motors()
        print("Exiting main program.")
