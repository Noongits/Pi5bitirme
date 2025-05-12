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

    navigation_thread = threading.Thread(target=navigate, daemon=True)
    navigation_thread.start()

    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        stop_motors()
        print("Exiting main program.")
