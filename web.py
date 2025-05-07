from flask import Flask, redirect, url_for,jsonify, Response
from flask_cors import CORS
import RPi.GPIO as GPIO
import threading
from motor_controller import *
import random
import state
import cv2

frame_lock = threading.Lock()

app = Flask(__name__)
CORS(app)

@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>Car Control</title>
    </head>
    <body>
        <h1>Control Car</h1>
        <button onclick="location.href='/forward'">Forward</button>
        <button onclick="location.href='/backward'">Backward</button>
        <button onclick="location.href='/left'">Left</button>
        <button onclick="location.href='/right'">Right</button>
        <button onclick="location.href='/stop'">Stop</button>
    </body>
    </html>
    """

@app.route('/forward')
def forward():
    move_forward()
    return redirect(url_for('index'))

@app.route('/backward')
def backward():
    move_backward()
    return redirect(url_for('index'))

@app.route('/left')
def left():
    turn_left()
    return redirect(url_for('index'))

@app.route('/right')
def right():
    turn_right(0.5)
    return redirect(url_for('index'))

@app.route('/stop')
def stop():
    stop_motors()
    return redirect(url_for('index'))

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an <img> tag."""
    return Response(_gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def _gen_frames():
    """Generator that yields JPEG-encoded frames from state.currentframe."""
    while True:
        # Wait until a frame is available
        with frame_lock:
            frame = state.currentframe.copy() if state.currentframe is not None else None

        if frame is None:
            # no frame yet, just loop
            continue

        # encode as JPEG
        success, buffer = cv2.imencode('.jpg', frame)
        if not success:
            continue

        jpg = buffer.tobytes()

        # yield frame in multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')





@app.route('/positions', methods=['GET'])
def get_positions():
    
    tag0 = {
        "x": state.tagarray[0, 0],
        "y": state.tagarray[0, 1],
        "z": state.tagarray[0, 2]
    }
    tag1 = {
        "x": state.tagarray[1, 0],
        "y": state.tagarray[1, 1],
        "z": state.tagarray[1, 2]
    }
    tag2 = {
        "x": state.tagarray[2, 0],
        "y": state.tagarray[2, 1],
        "z": state.tagarray[2, 2]
    }
    tag3 = {
        "x": state.tagarray[3, 0],
        "y": state.tagarray[3, 1],
        "z": state.tagarray[3, 2]
    }
    tag4 = {
        "x": state.tagarray[4, 0],
        "y": state.tagarray[4, 1],
        "z": state.tagarray[4, 2]
    }
    tag5 = {
        "x": state.tagarray[5, 0],
        "y": state.tagarray[5, 1],
        "z": state.tagarray[5, 2]
    }

    april_tag = {
        "x": state.estimated_position[0],
        "y": 0,
        "z": state.estimated_position[1]
    }
    imu = {
        "x": state.estimated_position[0],
        "y": 0,
        "z": state.estimated_position[1]
    }
    return jsonify({
        "aprilTagPosition": april_tag,
        "imuPosition": imu,
        "tag_0": tag0,
        "tag_1": tag1,
        "tag_2": tag2,
        "tag_3": tag3,
        "tag_4": tag4,
        "tag_5": tag5

    })


def run_control_server():
    """
    Starts the Flask control server. In a finally block, ensures that GPIO pins are cleaned up.
    """
    try:
        app.run(host='0.0.0.0', debug=False, use_reloader=False)
    finally:
        GPIO.cleanup()