import smbus
import time
import math

# MPU6050 Registers and addresses
MPU6050_ADDR   = 0x68   # I2C address of the MPU6050
PWR_MGMT_1     = 0x6B   # Power management register
SMPLRT_DIV     = 0x19   # Sample rate divider register
CONFIG         = 0x1A   # Configuration register
GYRO_CONFIG    = 0x1B   # Gyroscope configuration register
ACCEL_CONFIG   = 0x1C   # Accelerometer configuration register

# Starting register for accelerometer data (block read covers accelerometer, temperature, and gyroscope)
ACCEL_XOUT_H = 0x3B


# Initialize I2C bus (bus 1 on Raspberry Pi)
bus = smbus.SMBus(1)

def mpu6050_init():
    """Initialize the MPU6050 sensor."""
    # Wake up the sensor (exit sleep mode)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    time.sleep(0.1)
    # Set sample rate to 1kHz/(SMPLRT_DIV+1) => with SMPLRT_DIV=7, sample rate is 125 Hz
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    # Set digital low-pass filter (if desired, currently default)
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    # Set gyroscope full scale range to ±250°/s (131 LSB/°/s)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
    # Set accelerometer full scale range to ±2g (16384 LSB/g)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
    time.sleep(0.1)

def read_sensor_data():
    """
    Read accelerometer and gyroscope data using a single block read.
    Returns a dictionary with raw sensor values.
    """
    # Read 14 bytes starting from the ACCEL_XOUT_H register
    data = bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
    
    def combine_bytes(high, low):
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value
    
    sensor_data = {
        'ax': combine_bytes(data[0], data[1]),
        'ay': combine_bytes(data[2], data[3]),
        'az': combine_bytes(data[4], data[5]),
        # Temperature data is in data[6] and data[7] (not used here)
        'gx': combine_bytes(data[8], data[9]),
        'gy': combine_bytes(data[10], data[11]),
        'gz': combine_bytes(data[12], data[13])
    }
    
    return sensor_data

def calibrate(num_samples=2000):
    """
    Calibrate the sensor by computing the average offsets for the accelerometer and gyroscope.
    Ensure the sensor is stationary during calibration.
    
    Returns:
        Tuple of calibration offsets:
        (ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset)
    """
    print("Calibrating... Please keep the sensor still.")
    time.sleep(2)  # Allow sensor stabilization
    
    sum_ax = sum_ay = sum_az = 0
    sum_gx = sum_gy = sum_gz = 0
    
    for _ in range(num_samples):
        data = read_sensor_data()
        sum_ax += data['ax']
        sum_ay += data['ay']
        sum_az += data['az']
        sum_gx += data['gx']
        sum_gy += data['gy']
        sum_gz += data['gz']
        time.sleep(0.005)  # Short delay for quicker calibration
    
    # Compute average offsets
    ax_offset = sum_ax / num_samples
    ay_offset = sum_ay / num_samples
    az_offset = sum_az / num_samples
    gx_offset = sum_gx / num_samples
    gy_offset = sum_gy / num_samples
    gz_offset = sum_gz / num_samples
    
    # For the accelerometer, assume the z-axis experiences +1g at rest.
    # Adjust the z-axis offset by subtracting the 1g value (16384 for ±2g range).
    az_offset -= 16384
    
    print("Calibration complete.")
    return ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset

def main():
    mpu6050_init()
    ax_off, ay_off, az_off, gx_off, gy_off, gz_off = calibrate(num_samples=2000)
    
    print("Starting sensor readout and position estimation. Press Ctrl+C to exit.")
    
    # Use monotonic time for reliable dt calculation
    prev_time = time.monotonic()
    velocity = [0.0, 0.0, 0.0]  # Velocity in m/s for x, y, z
    position = [0.0, 0.0, 0.0]  # Position in m for x, y, z

    # Tunable damping coefficient (per second) to reduce velocity drift
    damping_coefficient = 0.5
    
    try:
        while True:
            current_time = time.monotonic()
            dt = current_time - prev_time
            prev_time = current_time
            
            # Calculate damping factor using exponential decay: factor = exp(-damping_coefficient * dt)
            damping_factor = math.exp(-damping_coefficient * dt)
            
            # Read sensor data as a block
            data = read_sensor_data()
            
            # Correct raw accelerometer data by subtracting calibration offsets
            ax = (data['ax'] - ax_off) / 16384.0 * 9.81  # Convert to m/s²
            ay = (data['ay'] - ay_off) / 16384.0 * 9.81
            az = (data['az'] - az_off) / 16384.0 * 9.81
            
            # Integrate acceleration into velocity, then apply damping
            velocity[0] = (velocity[0] + ax * dt) * damping_factor
            velocity[1] = (velocity[1] + ay * dt) * damping_factor
            velocity[2] = (velocity[2] + az * dt) * damping_factor
            
            # Integrate velocity to update position
            position[0] += velocity[0] * dt
            position[1] += velocity[1] * dt
            position[2] += velocity[2] * dt
            
            # Process gyroscope readings (convert raw data to °/s)
            gx = (data['gx'] - gx_off) / 131.0
            gy = (data['gy'] - gy_off) / 131.0
            gz = (data['gz'] - gz_off) / 131.0
            
            # Display the results
            print("Accelerometer (m/s²): X={:.2f}, Y={:.2f}, Z={:.2f}".format(ax, ay, az))
            print("Gyroscope (°/s):      X={:.2f}, Y={:.2f}, Z={:.2f}".format(gx, gy, gz))
            print("Estimated Position (m): X={:.3f}, Y={:.3f}, Z={:.3f}".format(
                position[0], position[1], position[2]
            ))
            print("-" * 40)
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("Exiting program.")

if __name__ == '__main__':
    main()
