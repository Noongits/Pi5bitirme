#!/usr/bin/env python3
import cv2
import apriltag
from picamera2 import Picamera2
import signal
import sys

# Initialize the Picamera2
picam2 = Picamera2()
# Create a preview configuration (you can adjust resolution as needed)
preview_config = picam2.create_preview_configuration(main={"size": (1280, 960)})
picam2.configure(preview_config)
picam2.start()

# Create an AprilTag detector instance
detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

# Graceful shutdown handler
def signal_handler(sig, frame):
    print("Exiting...")
    picam2.stop()
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

while True:
    # Capture a frame as a numpy array (BGR format)
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    # Convert the image to grayscale for tag detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect AprilTags in the grayscale image
    results = detector.detect(gray)
    
    # Process each detected tag
    for r in results:
        # Get the corners (as float, then convert to integer for drawing)
        corners = r.corners.astype(int)
        # Draw lines between the corners to form a bounding box
        cv2.line(frame, tuple(corners[0]), tuple(corners[1]), (0, 255, 0), 2)
        cv2.line(frame, tuple(corners[1]), tuple(corners[2]), (0, 255, 0), 2)
        cv2.line(frame, tuple(corners[2]), tuple(corners[3]), (0, 255, 0), 2)
        cv2.line(frame, tuple(corners[3]), tuple(corners[0]), (0, 255, 0), 2)
        
        # Draw the center of the tag
        center = (int(r.center[0]), int(r.center[1]))
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        
        # Annotate the tag with its ID
        cv2.putText(frame, f"ID: {r.tag_id} ", (center[0]-20, center[1]-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Display the annotated frame
    cv2.imshow("AprilTag Detection", frame)
    
    # Exit loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
picam2.stop()
cv2.destroyAllWindows()
