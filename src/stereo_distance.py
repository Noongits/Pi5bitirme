import cv2
import numpy as np
import variables

class StereoDistanceCalculator:
    def __init__(self, baseline_mm, focal_length_cam1_mm, focal_length_cam2_mm, 
                 sensor_width_cam1_mm, sensor_width_cam2_mm):
        """
        Initialize the stereo distance calculator for Raspberry Pi cameras
        
        Args:
            baseline_mm (float): Distance between camera positions in millimeters
            focal_length_cam1_mm (float): Camera 1 focal length in millimeters (2.1mm for Raspberry Pi Camera v2)
            focal_length_cam2_mm (float): Camera 2 focal length in millimeters (1.3mm for Raspberry Pi Camera v2)
            sensor_width_cam1_mm (float): Camera 1 sensor width in millimeters (3.674mm for Raspberry Pi Camera v2)
            sensor_width_cam2_mm (float): Camera 2 sensor width in millimeters (3.6mm for Raspberry Pi Camera v2)
        """
        self.baseline_mm = baseline_mm
        self.focal_length_cam1_mm = focal_length_cam1_mm
        self.focal_length_cam2_mm = focal_length_cam2_mm
        self.sensor_width_cam1_mm = sensor_width_cam1_mm
        self.sensor_width_cam2_mm = sensor_width_cam2_mm
        
        # Camera parameters for Raspberry Pi Camera v2
        # Resolution is typically 3280x2464 for Raspberry Pi Camera v2
        self.resolution_width = 1280
        self.resolution_height = 960
        
        # Calculate pixel sizes for each camera
        self.pixel_size_cam1_mm = sensor_width_cam1_mm / self.resolution_width
        self.pixel_size_cam2_mm = sensor_width_cam2_mm / self.resolution_width
        
        # Calculate focal length in pixels for both cameras
        self.focal_length_cam1_pixels = focal_length_cam1_mm / self.pixel_size_cam1_mm
        self.focal_length_cam2_pixels = focal_length_cam2_mm / self.pixel_size_cam2_mm
        
        # Use average focal length for distance calculations
        self.avg_focal_length_mm = (focal_length_cam1_mm + focal_length_cam2_mm) / 2
        self.avg_focal_length_pixels = (self.focal_length_cam1_pixels + self.focal_length_cam2_pixels) / 2

    def calculate_disparity(self, left_box, right_box):
        """
        Calculate disparity between two detections using their bounding boxes
        
        Args:
            left_box: (x1, y1, x2, y2) bounding box from left image
            right_box: (x1, y1, x2, y2) bounding box from right image
            
        Returns:
            float: Disparity in pixels
        """
        if left_box is None or right_box is None:
            return None
            
        # Calculate centers
        left_center_x = (left_box[0] + left_box[2]) / 2
        right_center_x = (right_box[0] + right_box[2]) / 2
        
        # Calculate horizontal disparity (difference in x positions)
        disparity = abs(left_center_x - right_center_x)
        return disparity

    def calculate_distance(self, disparity_pixels):
        """
        Calculate distance using triangulation with average focal length
        
        Args:
            disparity_pixels (float): Disparity in pixels
            
        Returns:
            float: Distance in meters
        """
        if disparity_pixels is None or disparity_pixels == 0:
            return None
            
        # Convert disparity to millimeters using average pixel size
        avg_pixel_size_mm = (self.pixel_size_cam1_mm + self.pixel_size_cam2_mm) / 2
        disparity_mm = disparity_pixels * avg_pixel_size_mm
        
        # Calculate distance using similar triangles with average focal length
        # distance = (baseline * focal_length) / disparity
        distance_mm = (self.baseline_mm * self.avg_focal_length_mm) / disparity_mm
        
        # Convert to meters
        distance_m = distance_mm / 1000
        
        return distance_m

    def process_images(self, left_box, right_box):
        """
        Process detection boxes and calculate distance to Eiffel Tower
        
        Args:
            left_box: (x1, y1, x2, y2) bounding box from left image
            right_box: (x1, y1, x2, y2) bounding box from right image
            
        Returns:
            dict: Results including distance and detection information
        """
        if left_box is None or right_box is None:
            return {
                'success': False,
                'error': 'One or both detection boxes are None'
            }
        
        # Calculate disparity
        disparity = self.calculate_disparity(left_box, right_box)
        
        # Calculate distance
        distance = self.calculate_distance(disparity)
        
        return {
            'success': True,
            'distance_meters': distance,
            'disparity_pixels': disparity,
            'left_detection': left_box,
            'right_detection': right_box
        }

def main():
    # Initialize calculator with Raspberry Pi camera parameters
    calculator = StereoDistanceCalculator(
        baseline_mm=50,  # 5 cm baseline
        focal_length_cam1_mm=2.1,  # Camera 1 focal length
        focal_length_cam2_mm=1.3,  # Camera 2 focal length
        sensor_width_cam1_mm=3.674,  # Camera 1 sensor width
        sensor_width_cam2_mm=3.6  # Camera 2 sensor width
    )

    try:
        # Get detection boxes from detect_and_crop
        if hasattr(variables, 'left_box') and hasattr(variables, 'right_box'):
            left_box = variables.left_box
            right_box = variables.right_box
            
            results = calculator.process_images(left_box, right_box)
            variables.eiffel_distance = results['distance_meters']
            
            if results['success']:
                print("\nStereo Distance Measured")
                # print(f"Distance to Eiffel Tower: {results['distance_meters']:.2f} meters")
                # print(f"Disparity: {results['disparity_pixels']:.2f} pixels")
                # print(f"Confidence: {results['confidence']:.2%}")
                # print("\nDetection details:")
                # print(f"Left image center: ({results['left_detection'][0]:.1f}, {results['left_detection'][1]:.1f})")
                # print(f"Right image center: ({results['right_detection'][0]:.1f}, {results['right_detection'][1]:.1f})")
            else:
                print(f"Error: {results['error']}")
        else:
            print("No detection boxes available from detect_and_crop")
            
    except Exception as e:
        print(f"Error processing images: {str(e)}")
