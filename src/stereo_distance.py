import cv2
import numpy as np
import torch
from ultralytics import YOLO
import math
import variables

class StereoDistanceCalculator:
    def __init__(self, baseline_mm=30, focal_length_cam1_mm=2.1, focal_length_cam2_mm=1.3, 
                 sensor_width_cam1_mm=3.674, sensor_width_cam2_mm=3.6):
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
        
        # Load YOLO model
        self.model = YOLO('src/models/bestbest_ncnn_model')
        
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

    def detect_eiffel_tower(self, image):
        """
        Detect Eiffel Tower in the image using YOLO
        
        Returns:
            tuple: (x_center, y_center, width, height) of the detection
        """
        results = self.model(image)
        
        # Get the first detection
        if len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            return ((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)
        return None

    def calculate_disparity(self, left_detection, right_detection):
        """
        Calculate disparity between two detections
        
        Args:
            left_detection: (x_center, y_center, width, height) from left image
            right_detection: (x_center, y_center, width, height) from right image
            
        Returns:
            float: Disparity in pixels
        """
        if left_detection is None or right_detection is None:
            return None
            
        # Calculate horizontal disparity (difference in x positions)
        disparity = abs(left_detection[0] - right_detection[0])
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

    def process_images(self, left_img, right_img):
        """
        Process two images and calculate distance to Eiffel Tower
        
        Args:
            left_image_path (str): Path to left image
            right_image_path (str): Path to right image
            
        Returns:
            dict: Results including distance and detection information
        """
        
        if left_img is None or right_img is None:
            raise ValueError("Could not load one or both images")
            
        # Detect Eiffel Tower in both images
        left_detection = self.detect_eiffel_tower(left_img)
        right_detection = self.detect_eiffel_tower(right_img)
        
        if left_detection is None or right_detection is None:
            return {
                'success': False,
                'error': 'Eiffel Tower not detected in one or both images'
            }
        
        # Calculate disparity
        disparity = self.calculate_disparity(left_detection, right_detection)
        
        # Calculate distance
        distance = self.calculate_distance(disparity)
        
        # Calculate confidence based on detection size
        left_confidence = left_detection[2] * left_detection[3] / (left_img.shape[0] * left_img.shape[1])
        right_confidence = right_detection[2] * right_detection[3] / (right_img.shape[0] * right_img.shape[1])
        confidence = (left_confidence + right_confidence) / 2
        
        # Draw detections on images
        left_img_with_box = self.draw_detection(left_img, left_detection)
        right_img_with_box = self.draw_detection(right_img, right_detection)
        
        # Save annotated images
        cv2.imwrite('dataset/left_detection.jpg', left_img_with_box)
        cv2.imwrite('dataset/right_detection.jpg', right_img_with_box)
        
        return {
            'success': True,
            'distance_meters': distance,
            'disparity_pixels': disparity,
            'confidence': confidence,
            'left_detection': left_detection,
            'right_detection': right_detection
        }

    def draw_detection(self, image, detection):
        """Draw detection box on image"""
        x_center, y_center, width, height = detection
        x1 = int(x_center - width/2)
        y1 = int(y_center - height/2)
        x2 = int(x_center + width/2)
        y2 = int(y_center + height/2)
        
        img_with_box = image.copy()
        cv2.rectangle(img_with_box, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img_with_box, f"Center: ({int(x_center)}, {int(y_center)})", 
                    (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return img_with_box

def main():
    # Initialize calculator with Raspberry Pi camera parameters
    calculator = StereoDistanceCalculator(
        baseline_mm=50,  # 5 cm baseline
        focal_length_cam1_mm=2.1,  # Camera 1 focal length
        focal_length_cam2_mm=1.3,  # Camera 2 focal length
        sensor_width_cam1_mm=3.674,  # Camera 1 sensor width
        sensor_width_cam2_mm=3.6  # Camera 2 sensor width
    )
    
    # Process images
    try:
        results = calculator.process_images(variables.leftcam, variables.rightcam)
        variables.eiffel_distance = results['distance_meters']
        
        if results['success']:
            print("\nStereo Distance Results:")
            print(f"Distance to Eiffel Tower: {results['distance_meters']:.2f} meters")
            print(f"Disparity: {results['disparity_pixels']:.2f} pixels")
            print(f"Confidence: {results['confidence']:.2%}")
            # print("\nDetection details:")
            # print(f"Left image center: ({results['left_detection'][0]:.1f}, {results['left_detection'][1]:.1f})")
            # print(f"Right image center: ({results['right_detection'][0]:.1f}, {results['right_detection'][1]:.1f})")
        else:
            print(f"Error: {results['error']}")
            
    except Exception as e:
        print(f"Error processing images: {str(e)}")
