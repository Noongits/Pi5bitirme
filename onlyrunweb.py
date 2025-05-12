import threading
from motor_controller import *
from sensor import sensor_loop
from web import run_control_server
import variables
import time
import threading
from web import run_control_server
control_thread = threading.Thread(target=run_control_server, daemon=True)
control_thread.start()