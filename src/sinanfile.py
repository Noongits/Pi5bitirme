from flask import Flask, jsonify
from flask_cors import CORS
import random

app = Flask(_name_)
CORS(app)

@app.route('/positions', methods=['GET'])
def get_positions():
    april_tag = {
        "x": round(random.uniform(-5, 5), 2),
        "y": 0,
        "z": round(random.uniform(-5, 5), 2)
    }
    imu = {
        "x": round(random.uniform(-5, 5), 2),
        "y": 0,
        "z": round(random.uniform(-5, 5), 2)
    }
    return jsonify({
        "aprilTagPosition": april_tag,
        "imuPosition": imu
    })

if _name_ == '_main_':
    app.run(host='0.0.0.0', port=5000)