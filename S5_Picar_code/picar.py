import front_wheels
import back_wheels
import time
from SunFounder_Line_Follower import Line_Follower
import ultrasonic_filtre

class Picar():

    max_speed = 70
    speed_car = 0

    # PID constants — à ajuster selon le comportement du robot
    KP = 15.0
    KI = 0.0
    KD = 8.0

    # Angle neutre des roues avant (tout droit)
    ANGLE_CENTER = 90

    # Limite max de correction angulaire (degrés de chaque côté)
    ANGLE_MAX = 30

    def __init__(self):
        self.front_wheels = front_wheels.Front_Wheels()
        self.back_wheels = back_wheels.Back_Wheels()
        self.ultrasonic_sensor = ultrasonic_filtre.UltrasonicSensor()
        self.lf = Line_Follower.Line_Follower()  # renommé pour éviter le conflit avec la méthode

        self._prev_error = 0.0
        self._integral = 0.0

    def forward(self, speed):
        self.back_wheels.forward()
        self.back_wheels.speed = speed
        self.speed_car = speed

    def backward(self, speed):
        self.back_wheels.backward()
        self.back_wheels.speed = speed

    def acceleration(self):
        while self.speed_car < self.max_speed:
            self.speed_car += 1
            self.forward(self.speed_car)
            time.sleep(0.1)

    def stop(self):
        self.back_wheels.stop()
        self.front_wheels.turn(self.ANGLE_CENTER)  # recentre les roues à l'arrêt

    def turn_while_moving(self, angle, speed, direction):
        if direction == "forward":
            self.front_wheels.turn(angle)
            self.forward(speed)
        elif direction == "backward":
            self.front_wheels.turn(angle)
            self.backward(speed)

    def line_following(self):
        """
        Suivi de ligne par PID en utilisant les valeurs analogiques brutes.
        read_position() retourne une erreur entre -2.0 (gauche) et +2.0 (droite).
        La correction est appliquée directement sur l'angle des roues avant.
        """
        error = self.lf.read_position()

        if error is None:
            # Ligne perdue : on arrête et on remet les roues droites
            print("ligne perdue")
            self.stop()
            self._integral = 0.0
            self._prev_error = 0.0
            return

        # Termes PID
        self._integral += error
        derivative = error - self._prev_error
        self._prev_error = error

        correction = (self.KP * error) + (self.KI * self._integral) + (self.KD * derivative)

        # Limite la correction pour ne pas dépasser les angles mécaniques
        correction = max(-self.ANGLE_MAX, min(self.ANGLE_MAX, correction))

        angle = self.ANGLE_CENTER + correction
        self.front_wheels.turn(angle)
        self.forward(self.speed_car)

        print(f"erreur: {error:.3f} | correction: {correction:.1f}° | angle: {angle:.1f}°")


def test():
    car = Picar()
    car.speed_car = 40
    car.forward(40)

    try:
        while True:
            car.line_following()
            time.sleep(0.05)  # boucle plus serrée = meilleure réponse PID
    finally:
        car.stop()


if __name__ == '__main__':
    test()