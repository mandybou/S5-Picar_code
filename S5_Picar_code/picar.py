import front_wheels
import back_wheels
import time
from SunFounder_Line_Follower import Line_Follower
import ultrasonic


class Picar():

    max_speed = 30
    speed_car = 0
    last_turn = 15
    lost_counter = 0
    obstacle_detected = False
    PATTERNS_CENTER = [
    [0, 0, 1, 0, 0],
    [0, 1, 1, 1, 0]
    #[0, 1, 0, 1, 0],
    #[1, 1, 1, 1, 0],
    #[0, 1, 1, 1, 1],
    #[1, 1, 1, 1, 1],
    ]
    PATTERNS_SLIGHT_LEFT = [[0, 1, 1, 0, 0], [0, 1, 0, 0, 0]]
    PATTERNS_HARD_LEFT = [[1, 0, 0, 0, 0], [1, 1, 0, 0, 0]]
    PATTERNS_SLIGHT_RIGHT = [[0, 0, 1, 1, 0], [0, 0, 0, 1, 0]]
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
        
    def acceleration(self, speed):
        while(self.speed_car < speed):
            self.speed_car += 2
            self.forward(self.speed_car)
            time.sleep(0.2)
            
    def decceleration(self, speed):
        while(self.speed_car > speed):
            self.speed_car -= 2
            self.forward(self.speed_car)
            time.sleep(0.2)
            
        
        
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
            self.lost_counter = 0
            if direction == "forward":
              self.turn_while_moving(0, self.max_speed, "forward")
            else:
              self.turn_while_moving(0, self.max_speed, "backward")

        elif status in self.PATTERNS_SLIGHT_LEFT:
            print("slight left")
            self.lost_counter = 0
            angle = -15 if direction == "forward" else 15
            self.turn_while_moving(angle, self.max_speed, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_HARD_LEFT:
            print("hard left")
            angle = -25 if direction == "forward" else 40
            self.turn_while_moving(angle, self.max_speed - 5, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_SLIGHT_RIGHT:
            print("slight right")
            self.lost_counter = 0
            angle = 15 if direction == "forward" else -15
            self.turn_while_moving(angle, self.max_speed, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_HARD_RIGHT:
            print("hard right")
            self.lost_counter = 0
            angle = 25 if direction == "forward" else -40
            self.turn_while_moving(angle, self.max_speed - 5, direction)
            self.last_turn = angle

        elif status in self.PATTERN_LOST:
            print(self.lost_counter)
            print("lost")
            if self.lost_counter >= 1:
              #recovery_direction = "backward" if direction == "forward" else "forward"
              self.decceleration(17)
              self.stop()
              #self.forward(0)
              
              time.sleep(0.4)
              #cmpt = 0
              #print(recovery_direction)
              while(self.line_follower.read_digital() in self.PATTERN_LOST):
                #cmpt+
                if self.speed_car <= 25:
                  self.speed_car +=2
                self.turn_while_moving((self.last_turn * -1), self.speed_car, "backward")
                time.sleep(0.4)
                
              
              self.stop()
              self.front_wheels.turn(90 + self.last_turn)
              #self.acceleration(25)
              time.sleep(0.1)
              self.lost_counter = 0
              #self.speed_car = self.max_speed
            else:
              #self.turn_while_moving((self.last_turn), self.max_speed - 7, direction)
              self.lost_counter+=1
            

    def obstacle_detection(self):
        distance = self.ultrasonic_sensor.read_distance()
        if distance is not None and distance < 30:
          self.obstacle_detected = True
        else:
          self.obstacle_detected = False



def test():
    car = Picar()
    car.forward(car.max_speed)
    state = 0

    try:
      while True:
        time.sleep(0.2)
        print(car.speed_car)
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
                distance = car.ultrasonic_sensor.read_distance()
                if distance is None:
                    state = 1
                elif distance <= 10:
                    car.stop()
                    time.sleep(0.5)
                    state = 2
                    
            case 2: #etat 2: reculer jusqua 30 cm 
                print("state 2")
                if car.ultrasonic_sensor.read_distance() < 30:
                    car.line_following("backward")
                    #time.sleep(0.2)
                else:
                    car.stop()
                    time.sleep(0.5)
                    print("state 3")
                    state = 3
            
            case 3:  #tourner a droite
                start = time.time()
                car.speed_car = round(car.max_speed * 0.8)
                print(start)
                while((time.time() - start) <= 2.75):
                  print("in while")
                  car.turn_while_moving(30, car.speed_car, "forward")
                  time.sleep(0.2)
                  
                car.stop()
                state = 4
                
            case 4: #contourner lobjet
                print("state 4")
                car.speed_car = round(car.max_speed * 0.8)
                print(car.line_follower.read_digital() in car.PATTERN_LOST)
                while(car.line_follower.read_digital() in car.PATTERN_LOST):
                  print(car.line_follower.read_digital())
                  print("state 4")
                  car.turn_while_moving(-15, car.speed_car, "forward")
                  time.sleep(0.2)
                  
                state = 0
                
                
              
                    

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