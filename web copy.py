from flask import Flask, redirect, url_for,jsonify
from flask_cors import CORS
import RPi.GPIO as GPIO
from motor_controller import *
import random
import variables


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

@app.route('/positions', methods=['GET'])
def get_positions():
    
    tag0 = {
        "x": variables.tagarray[0, 0],
        "y": variables.tagarray[0, 1],
        "z": variables.tagarray[0, 2]
    }
    april_tag = {
        "x": variables.estimated_position[0],
        "y": 0,
        "z": variables.estimated_position[1]
    }
    imu = {
        "x": variables.estimated_position[0],
        "y": 0,
        "z": variables.estimated_position[1]
    }
    return jsonify({
        "aprilTagPosition": april_tag,
        "imuPosition": imu,
        "tag_0": tag0
    })


def run_control_server():
    """
    Starts the Flask control server. In a finally block, ensures that GPIO pins are cleaned up.
    """
    try:
        app.run(host='0.0.0.0', debug=True, use_reloader=False)
    finally:
        GPIO.cleanup()