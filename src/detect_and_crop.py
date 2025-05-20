from ultralytics import YOLO
from PIL import Image, ImageDraw, ImageFont
import numpy as np
from pathlib import Path
import variables

# Paths
model_path = 'models/bestbest_ncnn_model'

# Load YOLO model
model = YOLO(model_path, task='detect')
model.conf = 0.75  # Confidence threshold

def draw_boxes(image_array, boxes):
    """Draw bounding boxes on the image using PIL."""
    # Convert numpy array to PIL Image
    image = Image.fromarray(image_array)
    draw = ImageDraw.Draw(image)
    
    # Try to load a font, fall back to default if not available
    try:
        font = ImageFont.truetype("arial.ttf", 20)
    except IOError:
        font = ImageFont.load_default()
    
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        conf = box.conf[0].item()
        cls = box.cls[0].item()
        
        # Draw rectangle
        draw.rectangle([(int(x1), int(y1)), (int(x2), int(y2))], 
                      outline=(0, 255, 0), width=2)
        
        # Add label
        label = f"Eiffel {conf:.2f}"
        draw.text((int(x1), int(y1) - 20), label, fill=(0, 255, 0), font=font)
    
    # Convert back to numpy array
    return np.array(image)

def process_frames(left_image, right_image):
    try:
        # Process left image (assuming it's already a numpy array)
        if not isinstance(left_image, np.ndarray):
            print("Left image must be a numpy array")
            return

        left_results = model(left_image)
        left_boxes = left_results[0].boxes
        
        if not left_boxes:
            print("No detections for left image")
            return

        # Find box with highest confidence for left image
        best_left_box = max(left_boxes, key=lambda b: b.conf[0].item())
        x1, y1, x2, y2 = map(int, best_left_box.xyxy[0].tolist())
        
        # Crop left image using numpy slicing
        left_crop = left_image[y1:y2, x1:x2]
        
        # Draw boxes on left image
        left_annotated = draw_boxes(left_image, left_boxes)
        
        # Process right image (assuming it's already a numpy array)
        if not isinstance(right_image, np.ndarray):
            print("Right image must be a numpy array")
            return

        right_results = model(right_image)
        right_boxes = right_results[0].boxes
        
        if not right_boxes:
            print("No detections for right image")
            return

        # Draw boxes on right image
        right_annotated = draw_boxes(right_image, right_boxes)
        
        # Save all outputs using PIL
        Image.fromarray(left_crop).save(variables.cropped_eiffel)
        Image.fromarray(left_annotated).save(variables.eiffel_left)
        Image.fromarray(right_annotated).save(variables.eiffel_right)
        
        print("Successfully processed and saved all images")
        return
        
    except Exception as e:
        print(f"Error processing images: {str(e)}")
        return

# # Process both frames
# print("Processing left and right frames...")
# process_frames(variables.leftcam, variables.rightcam) 
