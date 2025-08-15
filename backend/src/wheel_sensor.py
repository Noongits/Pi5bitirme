import RPi.GPIO as GPIO
import time
import variables
import math

class WheelSensor:
    
    def __init__(self,
                 sensor_pins: dict = {22: "sol arka", 15: "sağ ön"},
                 pulses_per_rev: int = 20,
                 diameter_mm: float = 65.0,
                 interval: float = 0.0005):
        self.sensor_pins = sensor_pins
        self.pulses_per_rev = pulses_per_rev
        self.dist_per_rev_m = math.pi * (diameter_mm / 1000.0) / 1.46
        self.interval = interval

        # initialize counts and distances for each sensor
        self.pulse_counts = {pin: 0 for pin in sensor_pins}
        self.total_distances = {pin: 0.0 for pin in sensor_pins}

        self._start_time = time.time()
        self._running = False

        GPIO.setmode(GPIO.BCM)
        for pin in sensor_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                pin,
                GPIO.FALLING,
                callback=self._pulse_callback,
                bouncetime=3
            )



    def _pulse_callback(self, channel):
        if channel in self.pulse_counts:
            self.pulse_counts[channel] += 1

    def start(self):
        """Start the measurement loop for all sensors."""
        self._running = True
        try:
            while self._running:
                time.sleep(self.interval)
                now = time.time()
                elapsed = now - self._start_time
                bothpins = 0
                pin22 =0
                pin15 = 0
                

                # compute and print only if movement detected
                for pin, label in self.sensor_pins.items():
                    count = self.pulse_counts[pin]
                    revolutions = count / self.pulses_per_rev
                    distance = revolutions * (self.dist_per_rev_m)
                    if distance > 0:
                        self.total_distances[pin] += distance
                        if pin == 22:
                            variables.LR_TOTAL_DISTANCE = self.total_distances[pin]
                            bothpins += 1
                            pin22 = distance
                        else:
                            variables.RF_TOTAL_DISTANCE = self.total_distances[pin]
                            bothpins += 1
                            pin15 = distance
                        # print distances in millimeters
                        #print(f"{label}: Last {elapsed:.3f}s → {distance*1000:.2f} mm, "
                              #f"Total → {self.total_distances[pin]*1000:.2f} mm"),
                                
                # your current world-space position:
                # e.g. a simple object with x, y, z attributes
                
                # your local move-vector
                
                pinsmal = select_bigger(pin15, pin22)
                move = (0, 0, pinsmal)

                
                if variables.currentlyForward:
                   print(f"pin small is {pinsmal} and self total distances {self.total_distances[22]}")
                   variables.pose_from_tyre_sensor = update_pose(variables.car_pose_tyresensor, pinsmal, variables.current_direction)
                   variables.pose_from_tyre_sensor = update_pose(variables.car_pose_tyresensor_fixed, pinsmal, variables.current_direction)
                elif variables.currentlyBackward:
                    print(f"pin small reverse is {pinsmal} and self total distances {self.total_distances[22]}")
                    variables.pose_from_tyre_sensor = update_pose(variables.car_pose_tyresensor, -pinsmal, variables.current_direction)
                    variables.pose_from_tyre_sensor = update_pose(variables.car_pose_tyresensor_fixed, -pinsmal, variables.current_direction)

                if False and bothpins == 2 and variables.currentlyForward:
                    
                    if variables.current_direction == 0:
                        vector3 = (0,0,pinsmal)
                        variables.pose_from_tyre_sensor += vector3
                    else:
                        if variables.current_direction == -90:
                            vector3 = (-pinsmal,0,0)
                            variables.pose_from_tyre_sensor += vector3

                # reset for next interval
                self.pulse_counts = {pin: 0 for pin in self.sensor_pins}
                self._start_time = now
        finally:
            GPIO.cleanup()

   
    def stop(self):
        """Signal the measurement loop to exit."""
        self._running = False

def update_pose(pose, distance, heading_deg):
        """
        pose: dict or simple object with .x and .z attributes
        distance: float, travelled since last update
        heading_deg: float, 0°→+Z, 90°→+X, -90°→-X
        """
        # 1. to radians
        #theta = math.radians(heading_deg)
        theta = heading_deg * math.pi / 180

        # 2. deltas
        dx = distance * math.sin(theta)
        dz = distance * math.cos(theta)

        # 3. update
        pose[0] -= dx
        pose[2] += dz

        return pose

def run_wheel_sensor():
    tester = WheelSensor()
    tester.start()

def select_smaller(a, b):
    if a < b:
        return a
    elif b < a:
        return b
    else:
        return a

def select_bigger(a, b):
    if a > b:
        return a
    elif b > a:
        return b
    else:
        return a

if __name__ == "__main__":
    run_wheel_sensor()
