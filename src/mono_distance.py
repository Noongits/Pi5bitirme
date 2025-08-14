import cv2
import numpy as np
import variables

class MonoDistanceCalculator:
    def __init__(self, focal_length_mm=3.4, sensor_width_mm=3.6):
        """
        Initialize the mono distance calculator for Raspberry Pi camera
        
        Args:
            focal_length_mm (float): Camera focal length in millimeters (3.4mm for Raspberry Pi Camera v2)
            sensor_width_mm (float): Camera sensor width in millimeters (3.6mm for Raspberry Pi Camera v2)
        """
        self.focal_length_mm = focal_length_mm
        self.sensor_width_mm = sensor_width_mm
        
        # Camera parameters for Raspberry Pi Camera v2
        self.resolution_width = 1280
        self.resolution_height = 960
        
        # Calculate pixel size
        self.pixel_size_mm = sensor_width_mm / self.resolution_width
        
        # Calculate focal length in pixels
        self.focal_length_pixels = focal_length_mm / self.pixel_size_mm
        self.focal_length_pixels = 713  # Calibrated value
        
        # Known height of the Eiffel Tower figure in meters
        self.known_height_m = 0.18  # 18 cm

    def calculate_distance(self, detection_box):
        """
        Calculate distance using the known height of the object and its apparent height in the image
        
        Args:
            detection_box: (x1, y1, x2, y2) bounding box from image
            
        Returns:
            float: Distance in meters
        """
        if detection_box is None:
            return None
            
        # Calculate the height of the object in pixels
        object_height_pixels = detection_box[3] - detection_box[1]  # y2 - y1
        
        # Convert pixel height to millimeters using pixel size
        object_height_mm = object_height_pixels * self.pixel_size_mm
        
        # Calculate distance using similar triangles
        # distance = (known_height * focal_length) / apparent_height
        distance_mm = (self.known_height_m * 1000 * self.focal_length_mm) / object_height_mm
        
        # Convert to meters
        distance_m = distance_mm / 1000
        
        return distance_m

    def process_image(self, detection_box):
        """
        Process detection box and calculate distance to Eiffel Tower figure
        
        Args:
            detection_box: (x1, y1, x2, y2) bounding box from image
            
        Returns:
            dict: Results including distance and detection information
        """
        if detection_box is None:
            return {
                'success': False,
                'error': 'Detection box is None'
            }
        
        # Calculate distance
        distance = self.calculate_distance(detection_box)
        
        return {
            'success': True,
            'distance_meters': distance,
            'detection': detection_box
        }

def main():
    # Initialize calculator with Raspberry Pi camera parameters
    calculator = MonoDistanceCalculator()

    try:
        # Get detection box from variables
        if hasattr(variables, 'left_box'):
            detection_box = variables.left_box
            
            results = calculator.process_image(detection_box)
            variables.eiffel_distance = results['distance_meters']
            
            #if results['success']:
                #print("\nMono Distance Measured: " + str(variables.eiffel_distance))
            if not results['success']:
                print(f"Error: {results['error']}")
        else:
            print("No detection box available from variables")
            
    except Exception as e:
        print(f"Error processing image: {str(e)}")
