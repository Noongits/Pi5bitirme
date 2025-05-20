import RPi.GPIO as GPIO
import time
import variables
import math

class SensorTest:
    def __init__(self,
                 sensor_pins: dict = {22: "sol arka", 15: "sağ ön"},
                 pulses_per_rev: int = 16,
                 diameter_mm: float = 65.0,
                 interval: float = 0.01):
        self.sensor_pins = sensor_pins
        self.pulses_per_rev = pulses_per_rev
        self.dist_per_rev_m = math.pi * (diameter_mm / 1000.0)
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
                bouncetime=20
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
                    distance = revolutions * self.dist_per_rev_m
                    if distance > 0:
                        self.total_distances[pin] += distance
                        if pin == 22:
                            variables.LR_TOTAL_DISTANCE = self.total_distances[pin]
                            bothpins += 1
                            pin22 = self.total_distances[pin]
                        else:
                            variables.RF_TOTAL_DISTANCE = self.total_distances[pin]
                            bothpins += 1
                            pin15 = self.total_distances[pin]
                        # print distances in millimeters
                        print(f"{label}: Last {elapsed:.3f}s → {distance*1000:.2f} mm, "
                              f"Total → {self.total_distances[pin]*1000:.2f} mm"),
                
                if bothpins == 2 and variables.currentlyForward:
                    pinsmal = select_smaller(pin15,pin22) 
                    if variables.current_direction == 0:
                        vector3 = (0,0,pinsmal)
                        variables.car_pose_tyresensor += vector3
                    else:
                        if variables.current_direction == -90:
                            vector3 = (-pinsmal,0,0)
                            variables.car_pose_tyresensor += vector3



                # reset for next interval
                self.pulse_counts = {pin: 0 for pin in self.sensor_pins}
                self._start_time = now
        finally:
            GPIO.cleanup()

    def stop(self):
        """Signal the measurement loop to exit."""
        self._running = False


def run_sensor_test():
    tester = SensorTest()
    tester.start()

def select_smaller(a, b):
    if a < b:
        return a
    elif b < a:
        return b
    else:
        return a

if __name__ == "__main__":
    run_sensor_test()
