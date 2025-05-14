import smbus
import time
import math
import json
import os
from dataclasses import dataclass
from typing import Tuple, Dict, Optional

@dataclass
class MPU6050Config:
    """Configuration parameters for MPU6050"""
    accel_range: int = 0  # 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
    gyro_range: int = 0   # 0: ±250°/s, 1: ±500°/s, 2: ±1000°/s, 3: ±2000°/s
    sample_rate: int = 7  # Sample rate = 1000/(1+SMPLRT_DIV)
    dlpf_config: int = 0  # Digital Low Pass Filter configuration

class MPU6050:
    # Register definitions
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    TEMP_OUT_H = 0x41
    GYRO_XOUT_H = 0x43
    WHO_AM_I = 0x75

    def __init__(self, bus_num: int = 1, address: int = 0x68):
        """Initialize MPU6050 with error handling"""
        self.address = address
        try:
            self.bus = smbus.SMBus(bus_num)
            self._check_connection()
        except Exception as e:
            raise RuntimeError(f"Failed to initialize I2C bus: {e}")

    def _check_connection(self) -> bool:
        """Verify sensor connection by reading WHO_AM_I register"""
        try:
            who_am_i = self.bus.read_byte_data(self.address, self.WHO_AM_I)
            if who_am_i != 0x68:
                raise RuntimeError(f"Invalid WHO_AM_I value: {who_am_i}")
            return True
        except Exception as e:
            raise RuntimeError(f"Failed to communicate with MPU6050: {e}")

    def initialize(self, config: Optional[MPU6050Config] = None) -> None:
        """Initialize sensor with configuration"""
        if config is None:
            config = MPU6050Config()

        try:
            # Wake up sensor
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
            time.sleep(0.1)

            # Configure sensor
            self.bus.write_byte_data(self.address, self.SMPLRT_DIV, config.sample_rate)
            self.bus.write_byte_data(self.address, self.CONFIG, config.dlpf_config)
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, config.gyro_range << 3)
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, config.accel_range << 3)
            time.sleep(0.1)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize MPU6050: {e}")

    def read_raw_data(self) -> Dict[str, int]:
        """Read raw sensor data with error handling"""
        try:
            data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 14)
            
            def combine_bytes(high: int, low: int) -> int:
                value = (high << 8) | low
                return value - 65536 if value > 32767 else value

            return {
                'ax': combine_bytes(data[0], data[1]),
                'ay': combine_bytes(data[2], data[3]),
                'az': combine_bytes(data[4], data[5]),
                'temp': combine_bytes(data[6], data[7]),
                'gx': combine_bytes(data[8], data[9]),
                'gy': combine_bytes(data[10], data[11]),
                'gz': combine_bytes(data[12], data[13])
            }
        except Exception as e:
            raise RuntimeError(f"Failed to read sensor data: {e}")

    def calibrate(self, num_samples: int = 2000, save_path: Optional[str] = None) -> Dict[str, float]:
        """Enhanced calibration with temperature compensation and data saving"""
        print("Calibrating... Please keep the sensor still.")
        time.sleep(2)

        sums = {axis: 0 for axis in ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp']}
        temps = []

        for _ in range(num_samples):
            data = self.read_raw_data()
            for key in sums:
                sums[key] += data[key]
            temps.append(data['temp'])
            time.sleep(0.005)

        # Calculate averages
        offsets = {key: sums[key] / num_samples for key in sums}
        
        # Temperature compensation
        avg_temp = sum(temps) / len(temps)
        temp_compensation = self._calculate_temp_compensation(avg_temp)
        
        # Apply temperature compensation
        for axis in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
            offsets[axis] += temp_compensation[axis]

        # Save calibration data if path provided
        if save_path:
            self._save_calibration(offsets, save_path)

        print("Calibration complete.")
        return offsets

    def _calculate_temp_compensation(self, temperature: float) -> Dict[str, float]:
        """Calculate temperature compensation factors"""
        # This is a simplified version - real implementation would use proper temperature compensation curves
        return {axis: 0.0 for axis in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']}

    def _save_calibration(self, offsets: Dict[str, float], path: str) -> None:
        """Save calibration data to file"""
        try:
            with open(path, 'w') as f:
                json.dump(offsets, f)
        except Exception as e:
            print(f"Warning: Failed to save calibration data: {e}")

    def load_calibration(self, path: str) -> Dict[str, float]:
        """Load calibration data from file"""
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            raise RuntimeError(f"Failed to load calibration data: {e}")

    def get_orientation(self, data: Dict[str, int], offsets: Dict[str, float]) -> Tuple[float, float, float]:
        """Calculate orientation using accelerometer data"""
        # Convert raw data to g
        ax = (data['ax'] - offsets['ax']) / 16384.0
        ay = (data['ay'] - offsets['ay']) / 16384.0
        az = (data['az'] - offsets['az']) / 16384.0

        # Calculate roll and pitch
        roll = math.atan2(ay, math.sqrt(ax * ax + az * az))
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        yaw = 0  # Yaw cannot be determined from accelerometer alone

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def main():
    try:
        # Initialize sensor
        mpu = MPU6050()
        mpu.initialize()

        # Calibrate or load calibration
        calib_file = "mpu6050_calibration.json"
        if os.path.exists(calib_file):
            offsets = mpu.load_calibration(calib_file)
        else:
            offsets = mpu.calibrate(save_path=calib_file)

        print("Starting sensor readout. Press Ctrl+C to exit.")
        
        while True:
            data = mpu.read_raw_data()
            roll, pitch, yaw = mpu.get_orientation(data, offsets)
            
            print(f"Orientation (degrees): Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}")
            print(f"Temperature: {data['temp']/340.0 + 36.53:.2f}°C")
            print("-" * 40)
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting program.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()