import mpu6050
import threading
import math
import time
from navigation import navigate
import variables

def sensor_loop():
    mpu6050.mpu6050_init()

    # === Calibration ===
    NUM_CALIBRATION_SAMPLES = 1000
    ax_off, ay_off, az_off, gx_off, gy_off, gz_off = mpu6050.calibrate(num_samples=NUM_CALIBRATION_SAMPLES)
    variables.calibrated = True
    print("calibrated mpu6050")

    # === Initialization ===
    prev_time = time.monotonic()
    velocity = [0.0, 0.0, 0.0]
    damping_coefficient = 0.5  # Adjust to tune drift resistance

    # Exponential Moving Average filter
    ema_alpha = 0.1
    filtered_acc = [0.0, 0.0, 0.0]

    # === Start Navigation Thread (test/demo only) ===
    move_thread = threading.Thread(target=navigate, args=(5, 10), daemon=True)
    #move_thread.start()

    while True:
        try:
            # === Time Delta ===
            current_time = time.monotonic()
            dt = current_time - prev_time
            prev_time = current_time
            damping_factor = math.exp(-damping_coefficient * dt)

            # === Read Raw Accelerometer ===
            data = mpu6050.read_sensor_data()
            raw_ax = (data['ax'] - ax_off) / 16384.0 * 9.81
            raw_ay = (data['ay'] - ay_off) / 16384.0 * 9.81
            raw_az = (data['az'] - az_off) / 16384.0 * 9.81

            # === Apply EMA filter ===
            filtered_acc[0] = ema_alpha * raw_ax + (1 - ema_alpha) * filtered_acc[0]
            filtered_acc[1] = ema_alpha * raw_ay + (1 - ema_alpha) * filtered_acc[1]
            filtered_acc[2] = ema_alpha * raw_az + (1 - ema_alpha) * filtered_acc[2]

            # === Reset velocity when not moving ===
            if not variables.currentlyForward and not variables.currentlyBackward:
                velocity[0] = velocity[1] = 0.0

            # === Integrate Acceleration to Estimate Velocity ===
            velocity[0] = (velocity[0] + filtered_acc[0] * dt) * damping_factor
            velocity[1] = (velocity[1] + filtered_acc[1] * dt) * damping_factor
            velocity[2] = (velocity[2] + filtered_acc[2] * dt) * damping_factor

            # === Integrate Velocity to Estimate Position ===
            variables.estimated_position[0] += velocity[0] * dt
            variables.estimated_position[1] += velocity[1] * dt
            # Optionally use Z too if needed

            time.sleep(0.001)

        except Exception as e:
            print(f"[Sensor Loop Error] {e}")
            time.sleep(0.1)
