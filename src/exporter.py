from ultralytics import YOLO


# Load the exported NCNN model
ncnn_model = YOLO("bestbest_ncnn_model")

# Run inference
results = ncnn_model("https://ultralytics.com/images/bus.jpg")