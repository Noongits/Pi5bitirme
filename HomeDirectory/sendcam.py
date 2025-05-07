from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import threading
import time

app = Flask(__name__)

# Initialize and configure the camera
picam2 = Picamera2()
config = picam2.create_preview_configuration()
picam2.configure(config)
picam2.start()

# Global variable to hold the latest frame and a lock for thread safety
latest_frame = None
frame_lock = threading.Lock()

def capture_frames():
    global latest_frame
    while True:
        # Capture, rotate, and resize frame
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.resize(frame, (320, 240))
        with frame_lock:
            latest_frame = frame
        # Small sleep to prevent maxing out CPU usage
        time.sleep(0.01)

# Start a background thread to capture frames continuously
capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

def gen_frames():
    while True:
        with frame_lock:
            frame = latest_frame
        if frame is None:
            continue
        # Use a lower JPEG quality to speed up encoding
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ret:
            continue
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Use threaded mode for the Flask server
    app.run(host='0.0.0.0', port=5050, threaded=True)
