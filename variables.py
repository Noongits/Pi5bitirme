import numpy as np
import threading

detected_tags = []
estimated_position = [0.0, 0.0, 0.0]
car_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
currentlyForward = False
currentlyBackward = False
currentlyRight = False
currentlyLeft = False
calibrated = True
currentframe = None
tagarray = np.zeros((16, 3), dtype=float)
lock = threading.Lock()
destionation = None
destination_reached = False

# Hardcoded AprilTag world coordinates (in meters)
APRILTAG_COORDS = {
    0: np.array([0.0, 0.0, 0.0]),
    2: np.array([2.0, 2.0, 2.0]),
    3: np.array([-3.0, -3.0, -3.0]),
    6: np.array([6.0, 6.0, 6.0]),
    8: np.array([8.0, 8.0, 8.0])}

destination = [-1, 2]