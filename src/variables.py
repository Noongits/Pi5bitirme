import numpy as np
import threading



LR_TOTAL_DISTANCE = 0.0
RF_TOTAL_DISTANCE = 0.0
detected_tags = []
APRILTAG_COORDS = {
    0: np.array([0.0, 0.0, 0.0]),
    2: np.array([0.0, 0.0, 2.0]),
    3: np.array([-3.0, -3.0, -3.0]),
    6: np.array([-2.0, 0.0, 3.0]),
    8: np.array([0.0, 0.0, 4.0])}

nav_mode = 1 # 0 = Straight line, 1 = Manhattan Navigation, 2 = Move to AprilTag, 3 = Move to landmark
nav_do_not_move = 1
estimated_position = [0.0, 0.0, 0.0]
car_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
car_pose_tyresensor = np.array([0.0, 0.0, 0.0])  # x, y, theta
tagarray = np.zeros((16, 3), dtype=float)
destination_reached = False
destination = [15.7, 12.3]
canstop = None

road_network = None
roadnet_height, roadnet_width = 20, 20
start_on_x = None
current_direction = 0

currentlyForward = False
currentlyBackward = False
currentlyRight = False
currentlyLeft = False

calibrated = True

left_frame_imm = None
right_frame_imm = None
tag_frame_left = None
tag_frame_right = None
leftcam = None
rightcam = None
leftlock = threading.Lock()
rightlock = threading.Lock()

eiffel_left = None
eiffel_right = None
cropped_eiffel = None

eiffel_location = [2.0, 0.0, 4.0]
eiffel_angle = None
eiffel_distance = None
