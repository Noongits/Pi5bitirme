from flask import Flask, redirect, url_for,jsonify, Response
from flask_cors import CORS
import RPi.GPIO as GPIO
from datetime import datetime
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



def gen_frames():
    while True:
        # Try to get the latest frame without blocking
        frame = None
        if frame_lock.acquire(blocking=False):
            frame = state.currentframe
            frame_lock.release()

        if frame is None:
            continue

        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    # Streams the video frames to the client
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/positions', methods=['GET'])
def get_positions():
    
    timestamp = datetime.now().isoformat()
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
        "timestamp": timestamp,
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