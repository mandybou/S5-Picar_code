import front_wheels
import back_wheels
import time
from SunFounder_Line_Follower import Line_Follower
import ultrasonic

class Picar():

    max_speed = 70
    speed_car = 0
    last_turn = 0
    obstacle_detected = False
    PATTERNS_CENTER = [
    [0, 0, 1, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 1, 0, 1, 0],
    [1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1],
    [1, 1, 1, 1, 1],
    ]
    PATTERNS_SLIGHT_LEFT = [[0, 1, 0, 0, 0], [0, 1, 1, 0, 0]]
    PATTERNS_HARD_LEFT = [[1, 0, 0, 0, 0], [1, 1, 0, 0, 0]]
    PATTERNS_SLIGHT_RIGHT = [[0, 0, 0, 1, 0], [0, 0, 1, 1, 0]]
    PATTERNS_HARD_RIGHT = [[0, 0, 0, 0, 1], [0, 0, 0, 1, 1]]
    PATTERN_LOST = [[0, 0, 0, 0, 0]]

    
    def __init__(self):
        self.front_wheels = front_wheels.Front_Wheels()
        self.back_wheels = back_wheels.Back_Wheels()
        self.ultrasonic_sensor = ultrasonic.UltrasonicSensor()
        self.line_follower = Line_Follower.Line_Follower()
    
    def forward(self, speed):
        self.back_wheels.forward()
        self.back_wheels.speed = speed
        self.speed_car = speed
        
    def backward(self, speed):
        self.back_wheels.backward()
        self.back_wheels.speed = speed
        
    def acceleration(self):
        while(self.speed_car < self.max_speed):
            self.speed_car += 1
            self.forward(self.speed_car)
            time.sleep(0.1)
        
        
    def stop(self):
      self.back_wheels.stop()
    
    def turn_while_moving(self, angle, speed, direction):
        if direction == "forward":
            print("Forward")
            self.front_wheels.turn(90 + angle)
            self.forward(speed)
        elif direction == "backward":
            self.front_wheels.turn(90 + angle)
            self.backward(speed)
            
    def line_following(self, direction="forward"):
        status = self.line_follower.read_digital()
        print(status)

        if status in self.PATTERNS_CENTER:
            print("center")
            if direction == "forward":
                self.turn_while_moving(0, self.speed_car, "forward")
            else:
                self.turn_while_moving(0, self.speed_car, "backward")

        elif status in self.PATTERNS_SLIGHT_LEFT:
            print("slight left")

            angle = -15 if direction == "forward" else 15
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_HARD_LEFT:
            print("left")
            angle = -25 if direction == "forward" else 25
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_SLIGHT_RIGHT:
            print("slight right")
            angle = 15 if direction == "forward" else -15
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_HARD_RIGHT:
            print("right")
            angle = 25 if direction == "forward" else -25
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERN_LOST:
            recovery_direction = "backward" if direction == "forward" else "forward"
            self.turn_while_moving((self.last_turn * -1), self.speed_car - 5, recovery_direction)
            time.sleep(0.2)

    def obstacle_detection(self):
        distance = self.ultrasonic_sensor.read_distance()
        if distance == None:
            self.obstacle_detected = False
        else:
            self.obstacle_detected = True



def test():
    car = Picar()
    car.forward(30)
    state = 0
    try:
      while True:
        time.sleep(0.2)
        match state:
            case 0: #etat 0 : Avancer et suivre la ligne
                print("state 0")
                car.obstacle_detection()
                car.line_following()
                #time.sleep(0.2)
                if car.obstacle_detected:
                    #car.stop()
                    #time.sleep(1)
                    state = 1

            case 1: #etat 1 : Si obstacle detecte, descelerrer jusqua 10 cm
                print("state 1")
                car.line_following()
                if car.ultrasonic_sensor.read_distance() <= 10:
                    car.stop()
                    time.sleep(1)
                    state = 2
            case 2: #etat 2: reculer jusqua 30 cm 
                print("state 2")
                if car.ultrasonic_sensor.read_distance() < 30:
                    car.line_following("backward")
                    #time.sleep(0.2)
                else:
                    car.stop()
                    time.sleep(1)
                    state = 3

        #etat 4: Evitement d'obstacle
        
    except KeyboardInterrupt:
        car.stop()
    
    #.acceleration()
    #car.forward(3)
    #time.sleep(5)
    #car.forward(50)
    #time.sleep(2)
    #car.forward(90)
    #time.sleep(5)
    
    #car.back_wheels.stop()
    
def stop_test():
  car = Picar()
  car.stop()



if __name__ == '__main__':
    try:
      test()
    finally:
      stop_test()