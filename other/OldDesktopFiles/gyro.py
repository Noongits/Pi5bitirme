import smbus
import time
import math

# Initialize I2C bus and MPU-6050 address
bus = smbus.SMBus(1)
address = 0x68

# Wake up MPU-6050 (exit sleep mode)
bus.write_byte_data(address, 0x6B, 0)

# Conversion factors
ACCEL_SCALE = 16384.0   # LSB/g for ±2g
GYRO_SCALE  = 131.0     # LSB/(°/s) for ±250°/s

def read_word(addr):
    """Reads two bytes from the device and converts them."""
    high = bus.read_byte_data(address, addr)
    low = bus.read_byte_data(address, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def get_accel_data():
    """Read accelerometer raw values for X, Y, and Z axes."""
    ax = read_word(0x3B)
    ay = read_word(0x3D)
    az = read_word(0x3F)
    return ax, ay, az

def get_gyro_data():
    """Read gyroscope raw values for X, Y, and Z axes."""
    gx = read_word(0x43)
    gy = read_word(0x45)
    gz = read_word(0x47)
    return gx, gy, gz

def get_temperature():
    """Read and convert the temperature from the sensor."""
    temp_row = read_word(0x41)
    return (temp_row / 340.0) + 36.53

# Dynamic calibration parameters
CALIBRATION_ALPHA = 0.01  # Smoothing factor (small value => slow adjustment)
GYRO_STATIONARY_THRESHOLD = 5.0  # °/s threshold to consider the sensor stationary
ACCEL_MAGNITUDE_TOLERANCE = 0.1  # Tolerance around 1g (in g) for accelerometer calibration

# Initialize offset values
gyro_offset = [0.0, 0.0, 0.0]
accel_offset = [0.0, 0.0, 0.0]

def update_offset(current_val, offset, alpha):
    """Update an offset value using an exponential moving average."""
    return (1 - alpha) * offset + alpha * current_val

if __name__ == "__main__":
    try:
        while True:
            # Get raw sensor data
            raw_accel = get_accel_data()
            raw_gyro  = get_gyro_data()

            # Convert raw values to physical units (g for accel, °/s for gyro)
            accel = [val / ACCEL_SCALE for val in raw_accel]
            gyro  = [val / GYRO_SCALE for val in raw_gyro]

            # Calculate the overall magnitudes
            gyro_magnitude = math.sqrt(sum(g**2 for g in gyro))
            accel_magnitude = math.sqrt(sum(a**2 for a in accel))
            
            # Dynamic calibration for the gyroscope:
            # If the sensor is nearly stationary (low angular velocity), update offsets.
            if all(abs(g) < GYRO_STATIONARY_THRESHOLD for g in gyro):
                for i in range(3):
                    gyro_offset[i] = update_offset(gyro[i], gyro_offset[i], CALIBRATION_ALPHA)
            
            # Dynamic calibration for the accelerometer:
            # If the total acceleration is close to 1g (only gravity), update offsets.
            if abs(accel_magnitude - 1.0) < ACCEL_MAGNITUDE_TOLERANCE:
                for i in range(3):
                    accel_offset[i] = update_offset(accel[i], accel_offset[i], CALIBRATION_ALPHA)
            
            # Apply calibration offsets to obtain calibrated readings
            calibrated_gyro  = [gyro[i] - gyro_offset[i] for i in range(3)]
            calibrated_accel = [accel[i] - accel_offset[i] for i in range(3)]
            
            # Get temperature reading
            temp = get_temperature()

            # Display calibrated sensor values
            print("Calibrated Accelerometer: X: {:.2f}g, Y: {:.2f}g, Z: {:.2f}g".format(*calibrated_accel))
            print("Calibrated Gyroscope:     X: {:.2f}°/s, Y: {:.2f}°/s, Z: {:.2f}°/s".format(*calibrated_gyro))
            print("Temperature:              {:.2f} °C".format(temp))
            print("-" * 40)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting the program.")
