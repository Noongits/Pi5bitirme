from flask import Flask, render_template, Response
import io
import time
from picamera import PiCamera

app = Flask(__name__)

# Initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 24
time.sleep(2)  # allow the camera to warm up

def gen_frames():
    """Video streaming generator function."""
    stream = io.BytesIO()
    # Capture frames continuously from the camera
    for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
        stream.seek(0)
        frame = stream.read()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        # Reset stream for next frame
        stream.seek(0)
        stream.truncate()

@app.route('/')
def index():
    """Home page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. This route returns a multipart response."""
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Run the Flask app on all available interfaces, so it is accessible in your network
    app.run(host='0.0.0.0', debug=True)
