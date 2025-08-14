import mpu6050
import time

from variables import estimated_orientation  # make this a [0.0, 0.0, 0.0] list in variables.py
import variables 

def gyroscope_loop():
    mpu6050.mpu6050_init()


    # === Calibration ===
    NUM_CALIBRATION_SAMPLES = 1000
    _, _, _, gx_off, gy_off, gz_off = mpu6050.calibrate(num_samples=NUM_CALIBRATION_SAMPLES)
    
    
    variables.imu_calibrated = True
    print("MPU6050 calibrated, starting orientation integration…")

    prev_t = time.monotonic()

    while True:
        try:
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            data = mpu6050.read_sensor_data()
            # Convert raw gyro (LSB) to °/s assuming ±250°/s full-scale (131 LSB per °/s)
            wx = (data['gx'] - gx_off) / 131.0
            wy = (data['gy'] - gy_off) / 131.0
            wz = (data['gz'] - gz_off) / 131.0

            # integrate into estimated_orientation = [roll, pitch, yaw]
            estimated_orientation[0] += wx * dt
            estimated_orientation[1] += wy * dt
            estimated_orientation[2] += wz * dt
            variables.current_direction = estimated_orientation[2]

            #print(f"Rotation → Roll: {estimated_orientation[0]:.2f}°, "
            #      f"Pitch: {estimated_orientation[1]:.2f}°, "
            #      f"Yaw: {estimated_orientation[2]:.2f}°")

            time.sleep(0.01)

        except Exception as e:
            print(f"[Sensor Loop Error] {e}")
            time.sleep(0.1)
