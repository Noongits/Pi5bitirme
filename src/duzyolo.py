#!/usr/bin/env python3
import time
import cv2
import numpy as np
from picamera2 import Picamera2

def main():
    # --- initialize camera ---
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # let exposure/WB settle

    # HSV range for “gray” (low saturation, mid-value)
    lower_gray = np.array([0,   0,  50])   # H:0–180, S:0–60, V:50–200
    upper_gray = np.array([180, 60, 200])

    try:
        while True:
            # 1) grab RGB frame
            rgb = picam2.capture_array()

            # 2) convert to OpenCV’s BGR order
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # 3) rotate the BGR image 180°
            bgr = cv2.rotate(bgr, cv2.ROTATE_180)

            # 4) threshold for gray in HSV
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_gray, upper_gray)

            # 5) find contours and draw the biggest one
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 6) display
            cv2.imshow("Gray Object Detection (Fixed Colors)", bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cv2.destroyAllWindows()
        picam2.stop()

if __name__ == "__main__":
    main()
