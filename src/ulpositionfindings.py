import cv2
import numpy as np
import time
from picamera2 import Picamera2
import os
import json
from datetime import datetime

class StereoCameraCalibrator:
    def _init_(self, chessboard_size=(9, 6), square_size=0.025):
        """
        Initialize the stereo camera calibrator
        
        Args:
            chessboard_size (tuple): Number of internal corners in the chessboard (width, height)
            square_size (float): Size of each square in meters
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ..., (8,5,0)
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size  # Scale to real world units
        
        # Arrays to store object points and image points
        self.objpoints = []  # 3D points in real world space
        self.imgpoints_left = []  # 2D points in left image plane
        self.imgpoints_right = []  # 2D points in right image plane
        
        # Initialize cameras
        self.left_camera = Picamera2(0)
        self.right_camera = Picamera2(1)
        
        # Configure cameras
        self._configure_cameras()
        
        # Create calibration directory
        self.calibration_dir = "camera_calibration"
        os.makedirs(self.calibration_dir, exist_ok=True)

    def _configure_cameras(self):
        """Configure both cameras with optimal settings for calibration"""
        config = {
            "format": "RGB888",
            "size": (1920, 1080),  # Full HD resolution
            "buffer_count": 4
        }
        
        # Configure and start both cameras
        self.left_camera.configure(self.left_camera.create_preview_configuration(**config))
        self.right_camera.configure(self.right_camera.create_preview_configuration(**config))
        
        self.left_camera.start()
        self.right_camera.start()
        
        # Allow cameras to warm up
        time.sleep(2)

    def capture_calibration_images(self, num_images=20, delay=2):
        """
        Capture calibration images from both cameras
        
        Args:
            num_images (int): Number of calibration images to capture
            delay (int): Delay between captures in seconds
        """
        print(f"Capturing {num_images} calibration images...")
        print("Move the chessboard to different positions and angles")
        
        captured = 0
        while captured < num_images:
            # Capture images
            left_img = self.left_camera.capture_array()
            right_img = self.right_camera.capture_array()
            
            # Convert to grayscale
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_RGB2GRAY)
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_RGB2GRAY)
            
            # Find chessboard corners
            left_ret, left_corners = cv2.findChessboardCorners(left_gray, self.chessboard_size, None)
            right_ret, right_corners = cv2.findChessboardCorners(right_gray, self.chessboard_size, None)
            
            # If found, refine corner positions and save
            if left_ret and right_ret:
                # Refine corner positions
                left_corners = cv2.cornerSubPix(left_gray, left_corners, (11, 11), (-1, -1), self.criteria)
                right_corners = cv2.cornerSubPix(right_gray, right_corners, (11, 11), (-1, -1), self.criteria)
                
                # Draw and display corners
                cv2.drawChessboardCorners(left_img, self.chessboard_size, left_corners, left_ret)
                cv2.drawChessboardCorners(right_img, self.chessboard_size, right_corners, right_ret)
                
                # Save images
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"{self.calibration_dir}/left_{timestamp}.jpg", left_img)
                cv2.imwrite(f"{self.calibration_dir}/right_{timestamp}.jpg", right_img)
                
                # Store points
                self.objpoints.append(self.objp)
                self.imgpoints_left.append(left_corners)
                self.imgpoints_right.append(right_corners)
                
                captured += 1
                print(f"Captured image pair {captured}/{num_images}")
                
                # Display preview
                cv2.imshow('Left Camera', left_img)
                cv2.imshow('Right Camera', right_img)
                cv2.waitKey(1000)  # Show for 1 second
            
            time.sleep(delay)
        
        cv2.destroyAllWindows()
        print("Calibration image capture complete!")

    def calibrate_cameras(self):
        """Perform stereo camera calibration"""
        print("Starting camera calibration...")
        
        # Calibrate individual cameras
        left_ret, left_mtx, left_dist, left_rvecs, left_tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_left, (1920, 1080), None, None)
        
        right_ret, right_mtx, right_dist, right_rvecs, right_tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_right, (1920, 1080), None, None)
        
        # Stereo calibration
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        
        criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Perform stereo calibration
        ret_stereo, left_mtx, left_dist, right_mtx, right_dist, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_left, self.imgpoints_right,
            left_mtx, left_dist, right_mtx, right_dist,
            (1920, 1080), criteria=criteria_stereo, flags=flags)
        
        # Save calibration results
        calibration_data = {
            'left_camera_matrix': left_mtx.tolist(),
            'left_distortion': left_dist.tolist(),
            'right_camera_matrix': right_mtx.tolist(),
            'right_distortion': right_dist.tolist(),
            'rotation_matrix': R.tolist(),
            'translation_vector': T.tolist(),
            'essential_matrix': E.tolist(),
            'fundamental_matrix': F.tolist(),
            'reprojection_error': float(ret_stereo)
        }
        
        with open(f"{self.calibration_dir}/stereo_calibration.json", 'w') as f:
            json.dump(calibration_data, f, indent=4)
        
        print("Calibration complete! Results saved to stereo_calibration.json")
        print(f"Reprojection error: {ret_stereo}")
        
        return calibration_data

    def cleanup(self):
        """Clean up camera resources"""
        self.left_camera.stop()
        self.right_camera.stop()

def main():
    # Create calibrator instance
    calibrator = StereoCameraCalibrator(chessboard_size=(9, 6), square_size=0.025)
    
    try:
        # Capture calibration images
        calibrator.capture_calibration_images(num_images=20, delay=2)
        
        # Perform calibration
        calibration_data = calibrator.calibrate_cameras()
        
        # Print some key information
        print("\nCalibration Summary:")
        print(f"Baseline (distance between cameras): {np.linalg.norm(calibration_data['translation_vector']):.3f} meters")
        print(f"Left camera focal length: {calibration_data['left_camera_matrix'][0][0]:.2f} pixels")
        print(f"Right camera focal length: {calibration_data['right_camera_matrix'][0][0]:.2f} pixels")
        
    finally:
        # Clean up
        calibrator.cleanup()

main()