import cv2
import numpy as np
import glob
import os

# === USER PARAMETERS ===
# Try different chessboard sizes - we'll attempt each one
chessboard_sizes = [
    (9, 6),   # Original size
    (8, 6),   # Common alternative
    (7, 6),   # Another common size
    (9, 7),   # Another possibility
    (8, 7)    # Another possibility
]
square_size = 1.0  # in any units (e.g., cm)
left_image_dir = 'board_left_cam'
right_image_dir = 'board_right_cam'
image_format = 'jpg'
debug_mode = True  # Set to True to save debug images
debug_output_dir = 'calibration_debug'  # Directory to save debug images

# Create debug output directory if it doesn't exist
if debug_mode and not os.path.exists(debug_output_dir):
    os.makedirs(debug_output_dir)

# === Prepare object points for each chessboard size ===
objpoints_dict = {}
for chessboard_size in chessboard_sizes:
    objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    objpoints_dict[chessboard_size] = objp

# === Collect image paths ===
left_images = sorted(glob.glob(os.path.join(left_image_dir, f'*.{image_format}')))
right_images = sorted(glob.glob(os.path.join(right_image_dir, f'*.{image_format}')))

assert len(left_images) == len(right_images), "Mismatched number of left/right images"

# === Detect corners ===
print("\nProcessing image pairs...")
best_size = None
best_matches = 0
best_objpoints = []
best_imgpoints_left = []
best_imgpoints_right = []

for chessboard_size in chessboard_sizes:
    print(f"\nTrying chessboard size: {chessboard_size}")
    objpoints = []  # 3D points in real world space
    imgpoints_left = []  # 2D points in left images
    imgpoints_right = []  # 2D points in right images
    
    for idx, (left_path, right_path) in enumerate(zip(left_images, right_images)):
        print(f"\nProcessing pair {idx + 1}/{len(left_images)}")
        print(f"Left image: {os.path.basename(left_path)}")
        print(f"Right image: {os.path.basename(right_path)}")
        
        img_left = cv2.imread(left_path)
        img_right = cv2.imread(right_path)
        
        if img_left is None or img_right is None:
            print(f"Error: Could not read one or both images")
            continue
            
        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        # Try to detect corners with different parameters
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, flags)
        ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, flags)

        if debug_mode:
            # Create copies of images for visualization
            vis_left = img_left.copy()
            vis_right = img_right.copy()
            
            # Draw corners if found
            if ret_left:
                cv2.drawChessboardCorners(vis_left, chessboard_size, corners_left, ret_left)
            if ret_right:
                cv2.drawChessboardCorners(vis_right, chessboard_size, corners_right, ret_right)
                
            # Add status text
            cv2.putText(vis_left, f"Size: {chessboard_size}, Corners: {ret_left}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if ret_left else (0, 0, 255), 2)
            cv2.putText(vis_right, f"Size: {chessboard_size}, Corners: {ret_right}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if ret_right else (0, 0, 255), 2)
            
            # Save debug images
            base_name = os.path.splitext(os.path.basename(left_path))[0]
            cv2.imwrite(os.path.join(debug_output_dir, f'{base_name}_left_{chessboard_size[0]}x{chessboard_size[1]}.jpg'), vis_left)
            cv2.imwrite(os.path.join(debug_output_dir, f'{base_name}_right_{chessboard_size[0]}x{chessboard_size[1]}.jpg'), vis_right)
            print(f"Saved debug images to {debug_output_dir}/")

        if ret_left and ret_right:
            print("✅ Successfully detected corners in both images")
            objpoints.append(objpoints_dict[chessboard_size])
            corners2_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1),
                                             criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            corners2_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1),
                                              criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            imgpoints_left.append(corners2_left)
            imgpoints_right.append(corners2_right)
        else:
            print("❌ Failed to detect corners:")
            if not ret_left:
                print("  - Left image: No corners found")
            if not ret_right:
                print("  - Right image: No corners found")
    
    # Keep track of which size gave us the most matches
    if len(objpoints) > best_matches:
        best_matches = len(objpoints)
        best_size = chessboard_size
        best_objpoints = objpoints
        best_imgpoints_left = imgpoints_left
        best_imgpoints_right = imgpoints_right

if best_matches == 0:
    print("\n❌ ERROR: No valid image pairs found for any chessboard size!")
    print("Please check that:")
    print("1. The chessboard is fully visible in the images")
    print("2. The chessboard pattern is clear and well-lit")
    print("3. The images are not blurry")
    print(f"\nDebug images have been saved to the '{debug_output_dir}' directory")
    print("Please examine these images to see why corner detection failed")
    exit(1)

print(f"\nBest chessboard size: {best_size}")
print(f"Found {best_matches} valid image pairs for calibration")

# Use the best results for calibration
objpoints = best_objpoints
imgpoints_left = best_imgpoints_left
imgpoints_right = best_imgpoints_right
chessboard_size = best_size

image_shape = gray_left.shape[::-1]  # (width, height)

# === Calibrate each camera individually ===
print("\nCalibrating LEFT camera...")
ret_left, mtx_left, dist_left, _, _ = cv2.calibrateCamera(
    objpoints, imgpoints_left, image_shape, None, None)

print("LEFT camera matrix:\n", mtx_left)
print("LEFT distortion coefficients:\n", dist_left)

print("\nCalibrating RIGHT camera...")
ret_right, mtx_right, dist_right, _, _ = cv2.calibrateCamera(
    objpoints, imgpoints_right, image_shape, None, None)

print("RIGHT camera matrix:\n", mtx_right)
print("RIGHT distortion coefficients:\n", dist_right)

# === Stereo calibration ===
print("\nRunning stereo calibration...")
flags = cv2.CALIB_FIX_INTRINSIC
criteria_stereo = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)

ret_stereo, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
    objpoints,
    imgpoints_left,
    imgpoints_right,
    mtx_left,
    dist_left,
    mtx_right,
    dist_right,
    image_shape,
    criteria=criteria_stereo,
    flags=flags
)

print("Stereo Calibration RMS Error:", ret_stereo)
print("Rotation matrix (R):\n", R)
print("Translation vector (T):\n", T)

# === Stereo Rectification ===
print("\nComputing rectification...")
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    mtx_left, dist_left, mtx_right, dist_right,
    image_shape, R, T, alpha=0
)

print("Disparity-to-depth mapping matrix (Q):\n", Q)

# Save calibration data
np.savez('stereo_calibration.npz',
         mtx_left=mtx_left, dist_left=dist_left,
         mtx_right=mtx_right, dist_right=dist_right,
         R=R, T=T, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q)

print("\n✅ Calibration complete. Parameters saved to 'stereo_calibration.npz'")
