from picamera2 import Picamera2
import cv2

# Initialize the camera
picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()

while True:
    # Capture an image as a NumPy array
    frame = picam2.capture_array()
    
    # Rotate the frame by 180 degrees (upside down)
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    
    # Display the rotated frame
    cv2.imshow("Camera Feed", frame)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
