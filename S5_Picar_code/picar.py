import time
from enum import Enum

import numpy as np
from SunFounder_Line_Follower import Line_Follower

import back_wheels
import front_wheels
import ultrasonic
import math

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
        print("integral = ", self._integral)
        derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = float(np.clip(output, *self.output_limits))

        self._prev_error = error
        self._prev_time = now
        return output


# ---------------------------------------------------------------------------
# Main car class
# ---------------------------------------------------------------------------

class Picar:
    CRUISE_SPEED    = 35   # normal forward cruising speed
    OBSTACLE_DIST   = 30   # cm - start slowing
    STOP_DIST       = 10   # cm - full stop
    BYPASS_DIST     = 23   # cm - back up until here
    LOST_PATIENCE   = 40    # iterations before declaring truly lost
    BRAKING_SCALE_CM = 13  # start here, adjust from observation

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
        self.pid = PID(kp=9.0, ki=1.0, kd=0.05, output_limits=(-65, 65))
        self._last_reliable_error = 0.0

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
        
    def _braking_distance(self) -> float:
        """Estimate cm traveled during _gradual_stop at the current speed."""
        return (self.speed / self.CRUISE_SPEED) * self.BRAKING_SCALE_CM

    def _steer(self, angle: float):
        """Translate a signed angle offset into a wheel angle (90 = straight)."""
        self.front_wheels.turn(int(90 + angle))
        
    def _tick_sharp_turn(self, lost_counter, error):
        # Optional: implement sharper turns by temporarily reducing speed on one wheel
        print("sharp_turn")
        if error > 0:
            self.back_wheels.set_speed_individual(
                self.speed - lost_counter, self.speed + lost_counter
            )
        elif error < 0:
            self.back_wheels.set_speed_individual(
                self.speed + lost_counter, self.speed - lost_counter
            )

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

    # def _read_line(self) -> tuple:
    #     return tuple(self.line_follower.read_digital())

    def _read_distance(self) -> float | None:
        return self.ultrasonic.read_distance()

    # def is_stop_pattern(self) -> bool:
    #     return self._read_line() in PATTERN_STOP

    def is_stop_pattern_analog(self, analog):
        THRESHOLD = 150  # adjust if needed
        black_count = sum(1 for v in analog if v < THRESHOLD)
        return black_count >= 4
        
    def is_lost_pattern_analog(self, analog):
        THRESHOLD = 150  # adjust if needed
        black_count = sum(1 for v in analog if v < THRESHOLD)
        return black_count == 0
    
    def _compute_line_error(self, analog_values):
        weights = [-2, -1, 0, 1, 2]

        # auto-normalization (no calibration needed)
        min_val = min(analog_values)
        max_val = max(analog_values)

        # no contrast ? no line
        if max_val - min_val < 50:
            return None

        weighted_sum = 0
        total = 0

        for i in range(5):
            # invert so black = high weight
            normalized = (max_val - analog_values[i]) / (max_val - min_val)
            weighted_sum += normalized * weights[i]
            total += normalized

        if total == 0:
            return None

        return weighted_sum / total

    # ------------------------------------------------------------------
    # PID line following
    # ------------------------------------------------------------------

    def line_following(self, direction: str = "forward") -> bool:
        backward = direction == "backward"

        analog = self.line_follower.read_analog()
        error = self._compute_line_error(analog)
        on_line = True
        if error is None:
            self._lost_counter += 1
            if self._lost_counter < self.LOST_PATIENCE:
                #error = self._last_error * (1 + 0.3 * self._lost_counter)
                #error = float(np.clip(self._last_error * (1 + 0.3 * self._lost_counter), -300, 300))
                on_line = False
                error = self._last_error
                print(error)
                self._tick_sharp_turn(self._lost_counter, error)
            else:
                return self._handle_lost(backward)
        else:
            self._lost_counter = 0
            self._last_reliable_error = error  # ? raw, unflipped, only when real

        # ---------- NORMAL CONTROL ----------
        if backward:
            print("backward")
            error = -error

        self._last_error = error
        angle = self.pid.compute(error)
        if not on_line:
          angle = (np.clip(angle * (1 + 0.3 * self._lost_counter), -65, 65))
          print(angle)
          
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
                    print("case 0")
                    car.target_speed = car.CRUISE_SPEED
                    analog = car.line_follower.read_analog()

                    if car.is_stop_pattern_analog(analog):
                        print("stop")
                        _gradual_stop(car)
                        time.sleep(10)
                        continue

                    distance = car._read_distance()
                    if distance is not None and distance < car.OBSTACLE_DIST:
                        state = 1
                        continue

                    lost = car.line_following()
                    #if lost:
                        #state = 5

                # -- 1: obstacle ahead - slow to STOP_DIST ------------------
                case 1:
                    car.line_following()
                    car.accel_state = AccelState.DECEL_FORWARD
                    car.target_speed = 19
                    print("case 1")
                    print(car.speed)
                    distance = car._read_distance()
                    if distance is None:
                        state = 0
                    else:
                        trigger_dist = car.STOP_DIST + car._braking_distance()
                        if distance <= trigger_dist:
                            _gradual_stop(car)
                            state = 2  # skip straight to backup state

                # -- 2: back up to BYPASS_DIST ------------------------------
                case 2:
                    print("case 2")
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
                        car._gradual_stop()
                        #time.sleep(0.2)
                        state = 3
                        
#                case 3:
#                  t1 = 0.0
#                  t2 = 2.0
#                  t3 = 2.5
#                  t4 = 5.0
#                  t_final = t4 + 1.0
#                  max_angle = 30
#              
#                  def smoothstep(t_start, t_end, x):
#                      k = max(0.0, min(1.0, (x - t_start) / (t_end - t_start)))
#                      return k * k * (3 - 2 * k)
#              
#                  def get_angle(t, sign):
#                      if 0 < t < t2:
#                          s = -smoothstep(t1, t2, t)
#                      elif t2 <= t <= t4:
#                          s = -(1.0 - smoothstep(t3, t4, t))
#                      else:
#                          s = 0.0
#                      return s * max_angle * sign
#              
#                  for sign in [1, -1]:  # Segment 1: gauche?droite, Segment 2: droite?gauche
#                      start = time.time()
#                      while True:
#                          t = time.time() - start
#                          if t > t_final:
#                              break
#              
#                          angle = get_angle(t, sign)
#              
#                          print(f"[sign={sign}] t: {t:.2f} | angle: {angle:.2f}")
#                          car.speed = min(car.speed + 2, car.CRUISE_SPEED)
#                          car._steer(angle)
#                          car._set_speed(car.speed)
#                          time.sleep(0.05)
#              
#                  car.pid._integral = 0.0
#                  car.accel_state = AccelState.ACCEL_FORWARD
#                  car.target_speed = car.CRUISE_SPEED
#                  state = 0



                # -- 3: swing right to clear obstacle -----------------------
                case 3:
                    start = time.time()
                    while time.time() - start <= 2.2:
                        print("case 3")
                        car.speed = min(car.speed + 4, car.CRUISE_SPEED)
                        car._steer(-30)
                        car._set_speed(car.speed)
                        time.sleep(0.2)
                    angle = 30
                    state = 4

                # -- 4: straighten until line reacquired --------------------
                case 4:
                    analog = car.line_follower.read_analog()
                    while car.is_lost_pattern_analog(analog):
                        print("case 4")
                        car.speed = max(car.speed - 1, car.CRUISE_SPEED - 3)
                        if angle < 20:
                          angle = angle + 5
                        car._steer(angle)
                        car._set_speed(car.speed)
                        time.sleep(0.2)
                        analog = car.line_follower.read_analog()
                    #car.pid.reset()
                    car.pid._integral = 0.0   # au lieu de reset()

                    car.accel_state = AccelState.ACCEL_FORWARD
                    car.target_speed = car.CRUISE_SPEED
                    car._set_speed(car.speed)
                    state = 0

                # -- 5: lost - slow down ------------------------------------
                case 5:
                    print("case 5")
                    car.accel_state = AccelState.DECEL_FORWARD
                    car.target_speed = 10
                    if car.speed <= car.target_speed:
                        car.stop()
                        time.sleep(0.5)
                        state = 6

                # -- 6: reverse to reacquire line ---------------------------
                case 6:
                    print("case 6")
                    analog = car.line_follower.read_analog()
                    if not car.is_lost_pattern_analog(analog):
                        car.stop()
                        time.sleep(0.5)
                        car.pid.reset()
                        state = 0
                    else:
                        car.accel_state = AccelState.ACCEL_BACKWARD
                        car.target_speed = 22
                        # Use last *reliable* error, negated for reverse direction
                        angle = float(np.clip(-car._last_reliable_error * 18, -65, 65))
                        car._steer(angle)
                        car._set_speed(car.speed, backward=True)

    except KeyboardInterrupt:
        car.stop()


def _gradual_stop(car: Picar):
    diff = (car.speed - 10) // 4
    for _ in range(4):
        car.speed = max(car.speed - diff, 0)
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