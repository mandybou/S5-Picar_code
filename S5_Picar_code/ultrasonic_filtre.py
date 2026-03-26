import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trig_pin=12, echo_pin=16, min_distance=3, max_distance=40):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.object_detected = False

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

        GPIO.output(self.trig_pin, False)
        print("Calibration.....")
        time.sleep(2)

    def read_distance(self):
        GPIO.output(self.trig_pin, True)
        time.sleep(0.06)
        GPIO.output(self.trig_pin, False)

        pulse_start = time.time()
        pulse_end = time.time()

        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()

        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 16666
        distance = round(distance, 2)

        if self.min_distance <= distance <= self.max_distance:
            self.object_detected = True
            return distance

        self.object_detected = False
        return None

    def cleanup(self):
        GPIO.cleanup((self.trig_pin, self.echo_pin))


def test():
    sensor = UltrasonicSensor()
    print("Placez un objet......")

    try:
        while True:
            distance = sensor.read_distance()

            if distance is not None:
                print("distance:", distance, "cm")
            elif sensor.object_detected is False:
                print("placez un objet....")

            time.sleep(1)

    except KeyboardInterrupt:
        sensor.cleanup()


if __name__ == "__main__":
    test()