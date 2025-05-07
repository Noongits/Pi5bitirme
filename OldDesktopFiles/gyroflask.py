import smbus
import time
import math
import threading
from flask import Flask, jsonify, render_template_string

app = Flask(__name__)

# Initialize I2C bus and MPU-6050 address
bus = smbus.SMBus(1)
address = 0x68

# Wake up MPU-6050 (exit sleep mode)
bus.write_byte_data(address, 0x6B, 0)

# Conversion factors
ACCEL_SCALE = 16384.0   # LSB/g for ±2g
GYRO_SCALE  = 131.0     # LSB/(°/s) for ±250°/s

def read_word(addr):
    """Read two bytes from the device and combine them."""
    high = bus.read_byte_data(address, addr)
    low = bus.read_byte_data(address, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def get_accel_data():
    """Read raw accelerometer values for X, Y, and Z."""
    ax = read_word(0x3B)
    ay = read_word(0x3D)
    az = read_word(0x3F)
    return ax, ay, az

def get_gyro_data():
    """Read raw gyroscope values for X, Y, and Z."""
    gx = read_word(0x43)
    gy = read_word(0x45)
    gz = read_word(0x47)
    return gx, gy, gz

def get_temperature():
    """Read and convert the temperature from the sensor."""
    temp_raw = read_word(0x41)
    return (temp_raw / 340.0) + 36.53

# Dynamic calibration parameters
CALIBRATION_ALPHA = 0.01           # Smoothing factor for calibration
GYRO_STATIONARY_THRESHOLD = 5.0      # °/s threshold to consider stationary
ACCEL_MAGNITUDE_TOLERANCE = 0.1      # Tolerance (in g) around 1g

# Calibration offsets (for gyroscope and accelerometer)
gyro_offset = [0.0, 0.0, 0.0]
accel_offset = [0.0, 0.0, 0.0]

def update_offset(current_val, offset, alpha):
    """Update offset using an exponential moving average."""
    return (1 - alpha) * offset + alpha * current_val

# Global variables for 2D position estimation (x and y in meters)
position = [0.0, 0.0]  # [x, y]
velocity = [0.0, 0.0]  # [vx, vy]

def sensor_loop():
    """Background loop that reads sensor data, calibrates, and integrates acceleration."""
    global gyro_offset, accel_offset, position, velocity
    dt = 0.1  # time step in seconds
    while True:
        # Read raw sensor data
        raw_accel = get_accel_data()   # (ax, ay, az) raw
        raw_gyro  = get_gyro_data()    # (gx, gy, gz) raw
        
        # Convert raw values to physical units
        accel = [val / ACCEL_SCALE for val in raw_accel]  # in g
        gyro  = [val / GYRO_SCALE for val in raw_gyro]      # in °/s
        
        # Compute overall magnitudes for calibration checks
        gyro_magnitude = math.sqrt(sum(g**2 for g in gyro))
        accel_magnitude = math.sqrt(sum(a**2 for a in accel))
        
        # Dynamic calibration for the gyroscope:
        if all(abs(g) < GYRO_STATIONARY_THRESHOLD for g in gyro):
            for i in range(3):
                gyro_offset[i] = update_offset(gyro[i], gyro_offset[i], CALIBRATION_ALPHA)
        
        # Dynamic calibration for the accelerometer:
        if abs(accel_magnitude - 1.0) < ACCEL_MAGNITUDE_TOLERANCE:
            for i in range(3):
                accel_offset[i] = update_offset(accel[i], accel_offset[i], CALIBRATION_ALPHA)
        
        # Apply calibration offsets
        calibrated_gyro = [gyro[i] - gyro_offset[i] for i in range(3)]
        calibrated_accel = [accel[i] - accel_offset[i] for i in range(3)]
        
        # For 2D position estimation, use X and Y axes only.
        # Convert acceleration from g to m/s² (multiply by 9.81)
        ax_m = calibrated_accel[0] * 9.81
        ay_m = calibrated_accel[1] * 9.81
        
        # --- Thresholding & Damping to Reduce Drift ---
        # Set a threshold below which acceleration is considered noise
        accel_threshold = 0.1  # m/s² threshold
        if abs(ax_m) < accel_threshold:
            ax_m = 0
        if abs(ay_m) < accel_threshold:
            ay_m = 0
        
        # Integrate acceleration to update velocity
        velocity[0] += ax_m * dt
        velocity[1] += ay_m * dt
        
        # Optionally apply damping to the velocity to counteract noise accumulation
        velocity_damping = 0.98  # Damping factor (values <1 gradually reduce velocity)
        velocity[0] *= velocity_damping
        velocity[1] *= velocity_damping
        
        # Update position based on velocity
        position[0] += velocity[0] * dt
        position[1] += velocity[1] * dt
        
        time.sleep(dt)

# Start the sensor reading loop in a separate thread
threading.Thread(target=sensor_loop, daemon=True).start()

# Flask routes

@app.route('/')
def index():
    # HTML page with an HTML5 canvas and JavaScript to fetch and display position
    html = """
    <!DOCTYPE html>
    <html>
    <head>
      <title>Position Estimate</title>
      <style>
        body { font-family: Arial, sans-serif; }
        #canvas { border: 1px solid #ccc; background: #f9f9f9; }
      </style>
    </head>
    <body>
      <h1>Position Estimate</h1>
      <canvas id="canvas" width="500" height="500"></canvas>
      <p>Position: (<span id="posX">0</span>, <span id="posY">0</span>) meters</p>
      <script>
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        const posXEl = document.getElementById('posX');
        const posYEl = document.getElementById('posY');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        
        function fetchPosition() {
          fetch('/position')
            .then(response => response.json())
            .then(data => {
              const posX = data.x;
              const posY = data.y;
              posXEl.textContent = posX.toFixed(2);
              posYEl.textContent = posY.toFixed(2);
              
              // Clear canvas and draw reference cross at the center
              ctx.clearRect(0, 0, canvas.width, canvas.height);
              ctx.beginPath();
              ctx.moveTo(centerX - 10, centerY);
              ctx.lineTo(centerX + 10, centerY);
              ctx.moveTo(centerX, centerY - 10);
              ctx.lineTo(centerX, centerY + 10);
              ctx.strokeStyle = '#aaa';
              ctx.stroke();
              
              // Scale position for visualization (adjust scale as needed)
              const scale = 50;  // 50 pixels per meter
              const drawX = centerX + posX * scale;
              const drawY = centerY - posY * scale;
              
              ctx.beginPath();
              ctx.arc(drawX, drawY, 5, 0, 2 * Math.PI);
              ctx.fillStyle = '#f00';
              ctx.fill();
            })
            .catch(err => console.error(err));
        }
        
        // Update the displayed position every 100ms
        setInterval(fetchPosition, 100);
      </script>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/position')
def get_position():
    # Return the current position as JSON
    return jsonify({"x": position[0], "y": position[1]})

if __name__ == '__main__':
    # Run Flask on all interfaces on port 5000
    app.run(host='0.0.0.0', port=5000)
