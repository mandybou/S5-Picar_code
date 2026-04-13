import time
from enum import Enum

import numpy as np
from SunFounder_Line_Follower import Line_Follower

import back_wheels
import front_wheels
import ultrasonic


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class AccelState(Enum):
    ACCEL_FORWARD  = 0
    ACCEL_BACKWARD = 1
    DECEL_FORWARD  = 2
    DECEL_BACKWARD = 3


class PID:
    """Simple discrete PID controller."""

    def __init__(self, kp: float, ki: float, kd: float, output_limits=(-65, 65)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()

    def compute(self, error: float) -> float:
        now = time.time()
        dt = now - self._prev_time
        if dt <= 0:
            dt = 1e-6

        self._integral += error * dt
        derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = float(np.clip(output, *self.output_limits))

        self._prev_error = error
        self._prev_time = now
        return output


# ---------------------------------------------------------------------------
# Sensor patterns ? weighted position error
# Sensors are indexed 0-4, left to right.
# We map each known pattern to a position in [-2, +2]:
#   negative = line is to the left  ? steer left
#   positive = line is to the right ? steer right
# ---------------------------------------------------------------------------

PATTERN_STOP = {
    (1, 1, 1, 1, 0),
    (0, 1, 1, 1, 1),
    (1, 1, 1, 1, 1),
}

PATTERN_LOST = (0, 0, 0, 0, 0)


# ---------------------------------------------------------------------------
# Main car class
# ---------------------------------------------------------------------------

class Picar:
    CRUISE_SPEED    = 20   # normal forward cruising speed
    OBSTACLE_DIST   = 30   # cm - start slowing
    STOP_DIST       = 10   # cm - full stop
    BYPASS_DIST     = 23   # cm - back up until here
    LOST_PATIENCE   = 5    # iterations before declaring truly lost

    def __init__(self):
        self.front_wheels     = front_wheels.Front_Wheels()
        self.back_wheels      = back_wheels.Back_Wheels()
        self.ultrasonic       = ultrasonic.UltrasonicSensor()
        self.line_follower    = Line_Follower.Line_Follower()

        self.speed            = 15
        self.accel_state      = AccelState.ACCEL_FORWARD
        self.target_speed     = self.CRUISE_SPEED

        self._lost_counter    = 0
        self._last_error      = 0.0   # used while lost to extrapolate

        # PID tuned for the hardware - adjust kp/ki/kd to taste
        #self.line_follower.calibrate() 
        self.pid = PID(kp=18.0, ki=0.5, kd=8.0, output_limits=(-65, 65))

    # ------------------------------------------------------------------
    # Low-level motion
    # ------------------------------------------------------------------

    def _set_speed(self, speed: int, backward: bool = False):
        self.speed = speed
        if backward:
            self.back_wheels.backward()
        else:
            self.back_wheels.forward()
        self.back_wheels.speed = speed

    def stop(self):
        self.back_wheels.stop()
        self.speed = 0

    def _steer(self, angle: float):
        """Translate a signed angle offset into a wheel angle (90 = straight)."""
        self.front_wheels.turn(int(90 + angle))

    # ------------------------------------------------------------------
    # Acceleration helpers
    # ------------------------------------------------------------------

    def _tick_accel(self):
        match self.accel_state:
            case AccelState.ACCEL_FORWARD:
                if self.speed < self.target_speed:
                    self._set_speed(self.speed + 1)
            case AccelState.ACCEL_BACKWARD:
                if self.speed < self.target_speed:
                    self._set_speed(self.speed + 1, backward=True)
            case AccelState.DECEL_FORWARD:
                if self.speed > self.target_speed:
                    self._set_speed(self.speed - 1)
            case AccelState.DECEL_BACKWARD:
                if self.speed > self.target_speed:
                    self._set_speed(self.speed - 1, backward=True)

    # ------------------------------------------------------------------
    # Sensor reads
    # ------------------------------------------------------------------

    def _read_line(self) -> tuple:
        return tuple(self.line_follower.read_digital())

    def _read_distance(self) -> float | None:
        return self.ultrasonic.read_distance()

    def is_stop_pattern(self) -> bool:
        return self._read_line() in PATTERN_STOP
    
    def _compute_line_error(self, analog_values):
        weights = [-2, -1, 0, 1, 2]
        weighted_sum = 0
        total = 0
    
        for i in range(5):
            raw = analog_values[i]
            white = self.line_follower._white_values[i]
            black = self.line_follower._black_values[i]
    
            span = white - black
            if span == 0:
                normalized = 0
            else:
                # 0.0 = sur le fond blanc, 1.0 = sur la ligne noire
                normalized = max(0.0, min(1.0, (raw - black) / span))
    
            weighted_sum += normalized * weights[i]
            total += normalized
    
        if total < 0.2:   # seuil : aucun capteur ne voit vraiment la ligne
            return None
    
        return weighted_sum / total

    # ------------------------------------------------------------------
    # PID line following
    # ------------------------------------------------------------------

    def line_following(self, direction: str = "forward") -> bool:
        backward = direction == "backward"
    
        # None means line is lost (all weights zero)
        analog = self.line_follower.read_analog()
        error = self._compute_line_error(analog)
    
        if error is None:
            return self._handle_lost(backward)
    
        self._lost_counter = 0
    
        # Check stop pattern - still needs digital read
        if self.is_stop_pattern():
            return False
    
        if backward:
            error = -error
    
        self._last_error = error
        angle = self.pid.compute(error)
        self._steer(angle)
    
        self.accel_state = AccelState.ACCEL_BACKWARD if backward else AccelState.ACCEL_FORWARD
        self._set_speed(self.speed, backward=backward)
        return False

    def _handle_lost(self, backward: bool) -> bool:
        """Extrapolate last known steering while lost; return True if timed out."""
        if self._lost_counter >= self.LOST_PATIENCE:
            self._lost_counter = 0
            self.pid.reset()
            return True

        # Keep turning harder in the last known direction
        extrapolated = self._last_error * (1 + 0.3 * self._lost_counter)
        extrapolated = float(np.clip(extrapolated, -65, 65))
        if backward:
            extrapolated = -extrapolated

        self._steer(extrapolated)
        self._set_speed(self.speed, backward=backward)
        self._lost_counter += 1
        return False


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

def run():
    car = Picar()
    state = 0

    try:
        while True:
            time.sleep(0.01)
            car._tick_accel()

            match state:
                # -- 0: follow line -----------------------------------------
                case 0:
                    if car.is_stop_pattern():
                        _gradual_stop(car)
                        time.sleep(10)
                        continue

                    distance = car._read_distance()
                    if distance is not None and distance < car.OBSTACLE_DIST:
                        state = 1
                        continue

                    lost = car.line_following()
                    if lost:
                        state = 5

                # -- 1: obstacle ahead - slow to STOP_DIST ------------------
                case 1:
                    car.line_following()
                    car.accel_state = AccelState.DECEL_FORWARD
                    car.target_speed = 19

                    distance = car._read_distance()
                    if distance is None:
                        state = 0
                    elif distance <= car.STOP_DIST:
                        car.stop()
                        time.sleep(1)
                        state = 2

                # -- 2: back up to BYPASS_DIST ------------------------------
                case 2:
                    distance = car._read_distance()
                    if distance is None:
                        state = 0
                        continue

                    if distance < car.BYPASS_DIST:
                        car.line_following("backward")
                        car.accel_state = AccelState.ACCEL_BACKWARD
                        car.target_speed = car.CRUISE_SPEED - 5
                    elif distance <= 30:
                        car.line_following("backward")
                        car.accel_state = AccelState.DECEL_BACKWARD
                        car.target_speed = 19
                    else:
                        car.stop()
                        time.sleep(1)
                        state = 3

                # -- 3: swing right to clear obstacle -----------------------
                case 3:
                    start = time.time()
                    while time.time() - start <= 2.5:
                        car.speed = min(car.speed + 1, car.CRUISE_SPEED)
                        car._steer(30)
                        car._set_speed(car.speed)
                        time.sleep(0.2)
                    state = 4

                # -- 4: straighten until line reacquired --------------------
                case 4:
                    confirmed = 0
                    while confirmed < 2:
                        car.speed = max(car.speed - 1, car.CRUISE_SPEED - 3)
                        car._steer(-15)
                        car._set_speed(car.speed)
                        time.sleep(0.2)
                        if car._read_line() not in (PATTERN_LOST,):
                            confirmed += 1
                        else:
                            confirmed = 0
                    car.pid.reset()
                    state = 0

                # -- 5: lost - slow down ------------------------------------
                case 5:
                    car.accel_state = AccelState.DECEL_FORWARD
                    car.target_speed = 10
                    if car.speed <= car.target_speed:
                        car.stop()
                        time.sleep(0.5)
                        state = 6

                # -- 6: reverse to reacquire line ---------------------------
                case 6:
                    status = car._read_line()
                    if status != PATTERN_LOST and any(status):
                        car.stop()
                        time.sleep(0.5)
                        car.pid.reset()
                        state = 0
                    else:
                        car.accel_state = AccelState.ACCEL_BACKWARD
                        car.target_speed = 22
                        # Reverse with mirrored last-known error
                        angle = float(np.clip(-car._last_error * 18, -65, 65))
                        car._steer(angle)
                        car._set_speed(car.speed, backward=True)

    except KeyboardInterrupt:
        car.stop()


def _gradual_stop(car: Picar):
    for _ in range(2):
        car.speed = max(car.speed - 2, 0)
        car._set_speed(car.speed)
        time.sleep(0.2)
    car.stop()


def reverse_to_stop():
    """Utility: reverse until a STOP pattern is detected, then brake."""
    car = Picar()
    car.stop()
    car.front_wheels.turn(90)
    speed = 22

    while car._read_line() not in PATTERN_STOP:
        car._set_speed(speed, backward=True)
        time.sleep(0.1)

    for _ in range(2):
        speed = max(speed - 2, 0)
        car._set_speed(speed, backward=True)
        time.sleep(0.4)

    car.stop()


if __name__ == "__main__":
    try:
        run()
    finally:
        Picar().stop()
