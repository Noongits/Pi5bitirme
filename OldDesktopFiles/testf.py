import cv2
import apriltag
import numpy as np
from picamera2 import Picamera2

def main():
    # Initialize and configure the Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration()
    picam2.configure(config)
    picam2.start()

    # Initialize the AprilTag detector for the "tag25h9" family.
    options = apriltag.DetectorOptions(families="tag25h9")
    detector = apriltag.Detector(options)

    while True:
        # Capture a frame as a numpy array.
        frame = picam2.capture_array()

        # Convert the captured frame to grayscale.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the grayscale image.
        tags = detector.detect(gray)

        # Process each detected tag.
        for tag in tags:
            # Draw a green polygon around the tag.
            pts = tag.corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            # Mark the center of the tag with a red dot.
            center = tuple(tag.center.astype(int))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Display the tag ID near the center.
            cv2.putText(frame, str(tag.tag_id), (center[0] + 10, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)

        # Show the frame with detected AprilTags.
        cv2.imshow('AprilTag Detection', frame)

        # Exit the loop if 'q' is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up and release resources.
    cv2.destroyAllWindows()
    picam2.stop()

if __name__ == '__main__':
    main()
