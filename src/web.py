from flask import Flask, redirect, url_for,jsonify, Response
from flask_cors import CORS
import RPi.GPIO as GPIO
from datetime import datetime
import threading
from motor_controller import *
import random
import io
import variables
import cv2
from PIL import Image

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


import cv2
import numpy as np

def gen_frames(target_size=(320, 240), jpeg_quality=70):
    """
    Generator that grabs the latest numpy frame from state.currentframe,
    flips it horizontally, downscales it to `target_size`, encodes as JPEG
    with Pillow, and yields it in multipart/x-mixed-replace format.
    """
    while True:
        frame = None
        # non-blocking lock check
        if variables.leftlock.acquire(blocking=False):
            frame = variables.leftcam
            variables.leftlock.release()

        if frame is None:
            continue

        # 1) Convert numpy array to PIL Image
        img = Image.fromarray(frame).convert("RGB")

        # 2) Flip horizontally
        #img = img.transpose(Image.FLIP_LEFT_RIGHT)
        # If you prefer a vertical flip instead, use:
        img = img.transpose(Image.FLIP_TOP_BOTTOM)

        # 3) Downscale to target_size using high-quality resampling
        img = img.resize(target_size, Image.ANTIALIAS)

        # 4) Encode as JPEG
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=jpeg_quality)
        frame_bytes = buf.getvalue()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
        )
        
def gen_framesbos():
    """
    Generator that yields a pre-encoded white JPEG frame forever,
    so you don’t re-encode on each iteration.
    """
    width, height = 320, 240
    # create a white PIL image once
    white_img = Image.new("RGB", (width, height), (100, 255, 255))
    buf = io.BytesIO()
    white_img.save(buf, format="JPEG", quality=70)
    white_jpeg = buf.getvalue()

    # just loop over the pre-encoded JPEG bytes
    while True:
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + white_jpeg + b'\r\n'
        )

@app.route('/video_feed')
def video_feed():
    # Streams the video frames to the client
    print("video feed istegi")
    if variables.leftcam is None:
        return Response(gen_framesbos(), mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/positions', methods=['GET'])
def get_positions():
    
    timestamp = datetime.now().isoformat()
    tag0 = {
        "x": variables.tagarray[0, 0],
        "y": variables.tagarray[0, 1],
        "z": variables.tagarray[0, 2]
    }
    tag1 = {
        "x": variables.tagarray[1, 0],
        "y": variables.tagarray[1, 1],
        "z": variables.tagarray[1, 2]
    }
    tag2 = {
        "x": variables.tagarray[2, 0],
        "y": variables.tagarray[2, 1],
        "z": variables.tagarray[2, 2]
    }
    tag3 = {
        "x": variables.tagarray[3, 0],
        "y": variables.tagarray[3, 1],
        "z": variables.tagarray[3, 2]
    }
    tag4 = {
        "x": variables.tagarray[4, 0],
        "y": variables.tagarray[4, 1],
        "z": variables.tagarray[4, 2]
    }
    tag5 = {
        "x": variables.tagarray[5, 0],
        "y": variables.tagarray[5, 1],
        "z": variables.tagarray[5, 2]
    }

    april_tag = {
        "x": variables.estimated_position[0],
        "y": 0,
        "z": variables.estimated_position[1]
    }
    sensor = {
        "x": variables.car_pose_tyresensor[0],
        "y": 0,
        "z": variables.car_pose_tyresensor[1]
    }
    imu = {
        "x": variables.estimated_position[0],
        "y": 0,
        "z": variables.estimated_position[1]
    }
    
    return jsonify({
        "timestamp": timestamp,
        "aprilTagPosition": april_tag,
        "imuPosition": imu,
        "sensorPosition": sensor,
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
        app.run(host='0.0.0.0', debug=True, use_reloader=False)
    finally:
        GPIO.cleanup()