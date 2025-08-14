import torch
from torchvision import transforms, models
from torchvision.models import ResNet18_Weights
from PIL import Image
import csv
from pathlib import Path
import numpy as np
import variables

# Define the classes (same as in train.py)
classes = ['0', '15', '30', '45', '60', '75', '105', '120', '150', '165']

# List of models to evaluate
MODEL_PATHS = [
    "models/angle_classifier.pth",
    "models/angle_classifier.pth"
]

# Global cache for loaded models
model_cache = {}

def load_model(model_path):
    # Check if model is already in cache

    print("angle started")
    if model_path in model_cache:
        return model_cache[model_path]
        
    # Load pretrained ResNet18 and modify for our classes
    model = models.resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
    num_ftrs = model.fc.in_features
    model.fc = torch.nn.Linear(num_ftrs, len(classes))
    
    # Load checkpoint and extract model state dict
    checkpoint = torch.load(model_path, map_location=torch.device('cpu'))
    
    # Handle different checkpoint formats based on model filename
    if 'angle_classifier.pth' in model_path:
        model.load_state_dict(checkpoint['model_state_dict'])
    elif 'angle_classifier_meh1.pth' in model_path:
        model.load_state_dict(checkpoint)
    else:
        # Default fallback - try both formats
        try:
            model.load_state_dict(checkpoint['model_state_dict'])
        except KeyError:
            model.load_state_dict(checkpoint)
    
    model.eval()
    
    # Store in cache
    model_cache[model_path] = model
    return model

def initialize_models():
    if model_cache == {}:
        for model_path in MODEL_PATHS:
            load_model(model_path)
        print("All models loaded and cached")

def predict_frame(frame, model, confidence_threshold=50.0):
    # Prepare the image
    transform = transforms.Compose([
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    
    # Convert frame to PIL Image if it's a numpy array
    if isinstance(frame, np.ndarray):
        image = Image.fromarray(frame)
    else:
        image = frame
    
    # Convert to RGB if needed
    if image.mode != 'RGB':
        image = image.convert('RGB')
    
    # Transform the image
    image_tensor = transform(image).unsqueeze(0)
    
    # Make prediction
    with torch.no_grad():
        outputs = model(image_tensor)
        _, predicted = outputs.max(1)
        probabilities = torch.nn.functional.softmax(outputs, dim=1)[0]
        
    predicted_class = classes[predicted.item()]
    confidence = probabilities[predicted].item() * 100
    
    # Return None if confidence is below threshold
    if confidence < confidence_threshold:
        return None, confidence
        
    return predicted_class, confidence

def process_frame_with_models(model_paths, frame, confidence_threshold=50.0):
    results = {}
    
    # Process each model
    for model_path in model_paths:
        # Get model from cache (will be loaded if not cached)
        model = load_model(model_path)
        
        try:
            predicted_class, confidence = predict_frame(frame, model, confidence_threshold)
            if predicted_class is None:
                results[model_path] = f"Low confidence ({confidence:.2f}%)"
                variables.eiffel_angle = None
            else:
                results[model_path] = f"{predicted_class} deg ({confidence:.2f}%)"
                variables.eiffel_angle = float(predicted_class)
        except Exception as e:
            print(f"Error in process_frame_with_models(): {str(e)}")
            results[model_path] = "Error"
    
    return results

def save_results_to_csv(results, output_file="prediction_results.csv"):
    # Get model names (without .pth extension)
    model_names = [Path(model_path).stem for model_path in results.keys()]
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write header
        header = ['Model', 'Prediction']
        writer.writerow(header)
        
        # Write results for each model
        for model_path, prediction in results.items():
            model_name = Path(model_path).stem
            writer.writerow([model_name, prediction])
        
    
    print(f"\nResults saved to {output_file}")

def main():
    # Initialize all models at startup
    initialize_models()
    
    try:
        # Load the image from file path
        if variables.cropped_eiffel is not None:
            try:
                image = Image.open(variables.cropped_eiffel)
                results = process_frame_with_models(MODEL_PATHS, image)
                #save_results_to_csv(results)
                # print(results)
            except Exception as e:
                print(f"Error loading image from {variables.cropped_eiffel}: {str(e)}")
        else:
            print("No cropped Eiffel Tower image available")

    except Exception as e:
        print(f"Error in angle_classification.main(): {str(e)}")

# Call initialize_models() when the module is imported
initialize_models()

# if __name__ == '__main__':
#     main()
