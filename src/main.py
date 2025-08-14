import threading
from web import run_control_server
from navigation import *
import wheel_sensor
import time
from get_frames import *
from gyroscope import *
import output

if __name__ == '__main__':

    output_thread = threading.Thread(target=output.main, daemon=True)
    output_thread.start()

    camera_thread = threading.Thread(target=capture_frames, daemon=True)
    camera_thread.start()

    control_thread = threading.Thread(target=run_control_server, daemon=True)
    control_thread.start()

    gyroscope_thread = threading.Thread(target=gyroscope_loop, daemon=True)
    gyroscope_thread.start()

    navigation_thread = threading.Thread(target=navigate, daemon=True)
    navigation_thread.start()

    wheel_sensor_thread = threading.Thread(target=wheel_sensor.run_wheel_sensor, daemon=True)
    wheel_sensor_thread.start()

    try:
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        stop_motors()
        print("Exiting main program.")
