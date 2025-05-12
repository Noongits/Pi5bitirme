import numpy as np
import threading

estimated_position = [0.0, 0.0, 0.0]
car_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
currentlyForward = False
currentlyBackward = False
currentlyRight = False
currentlyLeft = False
calibrated = False
currentframe = None
tagarray = np.zeros((15+1, 3), dtype=float)
lock = threading.Lock()
destination_reached = False
