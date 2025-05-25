import threading
from motor_controller import *
from web import run_control_server
from apriltag_detection import *
from localisation import *
from yolotestonlycam import *
from navigation2 import *
import sensortest
import time
from get_frames import *
from sensor import *
from localisation_eiffel import *

if __name__ == '__main__':
    control_thread = threading.Thread(target=run_control_server, daemon=True)
    control_thread.start()

    sensor_thread_mpu6050 = threading.Thread(target=sensor_loop, daemon=True)
    sensor_thread_mpu6050.start()

    navigation_thread = threading.Thread(target=navigate, daemon=True)
    navigation_thread.start()

    camera_thread = threading.Thread(target=capture_frames, daemon=True)
    camera_thread.start()

    sensor_thread = threading.Thread(target=sensortest.run_sensor_test, daemon=True)
    sensor_thread.start()

    eiffel_thread = threading.Thread(target=localise_with_eiffel, daemon=True)
    eiffel_thread.start()

    try:
        while True:
            print(f"current direction: {variables.current_direction}")
            time.sleep(1)
            
    except KeyboardInterrupt:
        stop_motors()
        print("Exiting main program.")
