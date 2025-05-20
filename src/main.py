import threading
from motor_controller import *
from web import run_control_server
from apriltag_detection import *
from localisation import *
from yolotestonlycam import *
from navigation2 import *
import sensortest
import time

if __name__ == '__main__':
    control_thread = threading.Thread(target=run_control_server, daemon=True)
    control_thread.start()

    yolo_thread = threading.Thread(target=main, daemon=True)
    yolo_thread.start()

    navigation_thread = threading.Thread(target=navigate, daemon=True)
    navigation_thread.start()

    sensor_thread = threading.Thread(
        target=sensortest.run_sensor_test,
        daemon=True
    )
    sensor_thread.start()


    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_motors()
        print("Exiting main program.")
