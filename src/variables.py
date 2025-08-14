import numpy as np
import threading

LR_TOTAL_DISTANCE = 0.0
RF_TOTAL_DISTANCE = 0.0
detected_tags = []
APRILTAG_COORDS = {
    0: np.array([0, 0.0, 2.2]),
    2: np.array([-0.6, 0.0, 2.2]),
    3: np.array([1.2, 0.0, 1.32]),
    5: np.array([1.2, 0.0, 1.2]),
    6: np.array([-0.11, 0.0, 1]),
    8: np.array([-2.25, 0.0, 1.32])}

donotmove = False
nav_mode = 1 # 0 = Straight line, 1 = Manhattan Navigation, 2 = Move to AprilTag, 3 = Move to landmark
nav_do_not_move = 1
estimated_position = [0.0, 0.0, 0.0]
estimated_orientation = [0.0, 0.0, 0.0]
car_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
car_pose_tyresensor = np.array([0.0, 0.0, 0.0])  # x, y, theta
car_pose_tyresensor_fixed = np.array([0.0, 0.0, 0.0])  # x, y, theta
tagarray = np.zeros((16, 3), dtype=float)
destination_reached = False
destination = [0.0, 0.0]
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
imu_calibrated = False

left_frame_imm = None
right_frame_imm = None
tag_frame_left = None
tag_frame_right = None
leftcam = None
rightcam = None
leftlock = threading.Lock()
rightlock = threading.Lock()

# Image paths for Eiffel Tower detection
eiffel_left = 'dataset/left_detection.jpg'
eiffel_right = 'dataset/right_detection.jpg'
cropped_eiffel = 'dataset/eiffel_cropped.jpg'

# Detection boxes for stereo distance calculation
left_box = None
right_box = None


eiffel_location = [0.195, 0.0, 2.12]
eiffel_angle = None
eiffel_distance = None
