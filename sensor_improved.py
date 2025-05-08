import mpu6050_improved
import threading
import math
import time
import numpy as np
from navigation import navigate
import state

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def update(self, measurement):
        # Prediction update
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        # Measurement update
        kalman_gain = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
        self.posteri_estimate = priori_estimate + kalman_gain * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - kalman_gain) * priori_error_estimate

        return self.posteri_estimate

def detect_stationary(acc_data, threshold=0.1):
    """Detect if the device is stationary based on acceleration variance"""
    return np.var(acc_data) < threshold

def sensor_loop():
    # Initialize MPU6050 with default configuration
    mpu = mpu6050_improved.MPU6050()
    mpu.initialize()

    # Load or perform calibration
    calib_file = "mpu6050_calibration.json"
    try:
        offsets = mpu.load_calibration(calib_file)
        print("Loaded existing calibration")
    except:
        print("Performing new calibration...")
        offsets = mpu.calibrate(save_path=calib_file)
    
    state.calibrated = True
    print("Calibration complete")

    # === Initialize Kalman Filters ===
    kf_x = KalmanFilter(process_variance=0.1, measurement_variance=0.5)
    kf_y = KalmanFilter(process_variance=0.1, measurement_variance=0.5)
    kf_z = KalmanFilter(process_variance=0.1, measurement_variance=0.5)

    # === Initialize Variables ===
    prev_time = time.monotonic()
    velocity = [0.0, 0.0, 0.0]
    position = [0.0, 0.0, 0.0]
    
    # Stationary detection window
    acc_window = []
    WINDOW_SIZE = 10
    
    # Complementary filter parameters
    alpha = 0.96  # Gyroscope trust factor
    beta = 0.04   # Accelerometer trust factor
    
    # Zero velocity update parameters
    ZUPT_THRESHOLD = 0.1
    last_stationary_time = time.monotonic()
    ZUPT_TIMEOUT = 0.5  # seconds

    while True:
        try:
            # === Time Delta ===
            current_time = time.monotonic()
            dt = current_time - prev_time
            prev_time = current_time

            # === Read Sensor Data ===
            data = mpu.read_raw_data()
            
            # Convert raw data to g
            ax = (data['ax'] - offsets['ax']) / 16384.0
            ay = (data['ay'] - offsets['ay']) / 16384.0
            az = (data['az'] - offsets['az']) / 16384.0
            
            # Apply Kalman filtering
            ax_filtered = kf_x.update(ax)
            ay_filtered = kf_y.update(ay)
            az_filtered = kf_z.update(az)
            
            # Update acceleration window for stationary detection
            acc_window.append([ax_filtered, ay_filtered, az_filtered])
            if len(acc_window) > WINDOW_SIZE:
                acc_window.pop(0)
            
            # Check if stationary
            is_stationary = detect_stationary(acc_window)
            
            # Zero Velocity Update (ZUPT)
            if is_stationary:
                last_stationary_time = current_time
                velocity = [0.0, 0.0, 0.0]
            elif current_time - last_stationary_time > ZUPT_TIMEOUT:
                # Only integrate if we're not in ZUPT
                velocity[0] += ax_filtered * dt
                velocity[1] += ay_filtered * dt
                velocity[2] += az_filtered * dt
                
                # Apply velocity damping
                damping = math.exp(-0.5 * dt)  # Adjust damping coefficient as needed
                velocity = [v * damping for v in velocity]
                
                # Update position
                position[0] += velocity[0] * dt
                position[1] += velocity[1] * dt
                position[2] += velocity[2] * dt
            
            # Update state
            state.estimated_position = position.copy()
            
            # Small sleep to prevent CPU overload
            time.sleep(0.001)

        except Exception as e:
            print(f"[Sensor Loop Error] {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    sensor_loop() 