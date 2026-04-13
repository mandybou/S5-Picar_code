import front_wheels
import back_wheels
import time
from SunFounder_Line_Follower import Line_Follower
import ultrasonic
import numpy as np


class Picar():
    max_speed = 20
    max_speed_acc = 20

    #0: acceleration avant, 1: acceleration arriere, 2: decceleration avant, 3: decceleration arriere
    acc_state = 0
    speed_car = 15
    last_turn = 15
    last_turn_state = 0
    lost_counter = 0
    angle = 0
    obstacle_detected = False
    cmpt = 0
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
    PATTERN_STOP = [[1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1],
    [1, 1, 1, 1, 1]]
    

    
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
        self.speed_car = speed
        
    def acceleration(self, speed, direction):
        #self.cmpt += 1
        if(self.speed_car < speed):
            self.cmpt = 0
            self.speed_car += 1
            if direction == "forward":
                self.forward(self.speed_car)
            elif direction == "backward":
                self.backward(self.speed_car)
            
    def decceleration(self, speed, direction):
        #self.cmpt += 1
        if(self.speed_car > speed):
            #self.cmpt = 0
            self.speed_car -= 1
            if direction == "forward":
                self.forward(self.speed_car)
            elif direction == "backward":
                self.backward(self.speed_car)
            
        
    def stop(self):
      self.speed_car = 15
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
        
            self.max_speed_acc = self.max_speed
            
            if self.angle < 2:
                self.angle += 2
            elif self.angle > -2:
                self.angle -= 2
            else:
                self.angle = 0
            
            if direction == "forward":
              self.acc_state = 0
              #self.angle = 0
              self.turn_while_moving(self.angle, self.speed_car, "forward")
            else:
              self.acc_state = 1
              #self.angle = 0
              self.turn_while_moving(self.angle, self.speed_car, "backward")

        elif status in self.PATTERNS_SLIGHT_LEFT:
            print("slight left")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed
            #angle = -15 if direction == "forward" else 15
            if direction == "forward":
                self.angle = np.clip(self.angle - 1, -17, 17)
                self.acc_state = 0
            else:
                #self.angle = np.clip(self.angle + 1, -3, 3)
                self.angle = 0
                self.acc_state = 1
            self.turn_while_moving(self.angle, self.speed_car, direction)
            self.last_turn = self.angle

        elif status in self.PATTERNS_HARD_LEFT:
            print("hard left")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed
            #self.angle
            if direction == "forward":
                if self.angle > 0:
                  self.angle = 0
                self.angle = np.clip(self.angle - 3, -65, 65)
                self.acc_state = 0
            else:
                #self.angle = np.clip(self.angle + 1, -5, 5)
                self.angle = 0
                self.acc_state = 1
            self.turn_while_moving(self.angle, self.speed_car, direction)
            self.last_turn = self.angle

        elif status in self.PATTERNS_SLIGHT_RIGHT:
            print("slight right")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed
            if direction == "forward":
                self.angle = np.clip(self.angle + 1, -17, 17)
                self.acc_state = 0
            else:
                #self.angle = np.clip(self.angle - 1, -3, 3)
                self.angle = 0
                self.acc_state = 1
            self.turn_while_moving(self.angle, self.speed_car, direction)
            self.last_turn = self.angle

        elif status in self.PATTERNS_HARD_RIGHT:
            print("hard right")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed
            if direction == "forward":
                if self.angle < 0:
                  self.angle = 0
                self.angle = np.clip(self.angle + 3, -65, 65)
                self.acc_state = 0
            else:
                #self.angle = np.clip(self.angle - 1, -3, 3)
                self.angle = 0
                self.acc_state = 1
            self.turn_while_moving(self.angle, self.speed_car, direction)
            self.last_turn = self.angle

        elif status in self.PATTERN_LOST:
            print(self.lost_counter)
            print("lost")
            if self.lost_counter >= 5:
                self.lost_counter = 0
                return 1
            else:
                #self.angle = 0
                if self.last_turn > 0:
                    self.last_turn += 5    
                else:
                    self.last_turn -= 5
                    
                self.angle =  self.last_turn
                self.turn_while_moving(self.angle, self.speed_car, direction)
                self.lost_counter+=1
                
        return 0
            

    def obstacle_detection(self):
        distance = self.ultrasonic_sensor.read_distance()
        if distance is not None and distance < 30:
          self.obstacle_detected = True
        else:
          self.obstacle_detected = False



def test():
    car = Picar()
    #car.forward(car.max_speed)
    state = 0

    try:
      while True:
        time.sleep(0.01)
        print(car.speed_car)
        
        
        print(car.angle)

        match car.acc_state:
            case 0:
                car.acceleration(car.max_speed_acc, "forward")
            case 1:
                car.acceleration(car.max_speed_acc, "backward")
            case 2:
                car.decceleration(car.max_speed_acc, "forward")
            case 3:
                car.decceleration(car.max_speed_acc, "backward")

        match state:
            case 0: #etat 0 : Avancer et suivre la ligne
                print("state 0")
                car.obstacle_detection()
                hasNoPath = car.line_following()
                #time.sleep(0.2)
                if car.obstacle_detected:
                    #car.stop()
                    #time.sleep(1)
                    state = 1

                if hasNoPath == 1:
                    state = 5
                if car.line_follower.read_digital() in car.PATTERN_STOP:
                    car.speed_car -=2
                    time.sleep(0.2)
                    car.speed_car -=2
                    time.sleep(0.2)
                    car.stop()
                    time.sleep(10)
                    

            case 1: #etat 1 : Si obstacle detecte, descelerrer jusqua 10 cm
                print("state 1")
                car.line_following()
                car.acc_state = 2
                car.max_speed_acc = 19
                distance = car.ultrasonic_sensor.read_distance()
                if distance is None:
                    state = 0
                elif distance <= 10:
                    car.stop()
                    time.sleep(1)
                    state = 2
                    
            case 2: #etat 2: reculer jusqua 30 cm 
                print("state 2")
                if car.ultrasonic_sensor.read_distance() < 23:
                    car.line_following("backward")
                    car.acc_state = 1
                    car.max_speed_acc = car.max_speed - 5
                    #time.sleep(0.2)
                elif car.ultrasonic_sensor.read_distance() >= 23 and car.ultrasonic_sensor.read_distance() <= 30:
                    car.line_following("backward")
                    car.acc_state = 3
                    car.max_speed_acc = 19
                else:
                    car.stop()
                    time.sleep(1)
                    print("state 3")
                    state = 3
            
            case 3:  #tourner a droite
                start = time.time()
                #car.speed_car = round(car.max_speed * 0.8)
                print(start)
                while((time.time() - start) <= 2.5):
                  print("in while")
                  if car.speed_car <= car.max_speed:
                    car.speed_car += 1
                    
                  car.turn_while_moving(30, car.speed_car, "forward")
                  time.sleep(0.2)
                  
                #car.stop()
                state = 4
                
            case 4: #contourner lobjet
                print("state 4")
                cmpt = 0
                while(cmpt < 2):
                  if car.speed_car >= 23:
                    car.speed_car -= 1
                    
                  print(car.line_follower.read_digital())
                  print("state 4")
                  car.turn_while_moving(-15, car.speed_car, "forward")
                  time.sleep(0.2)
                  if (car.line_follower.read_digital() not in car.PATTERN_LOST):
                    cmpt +=1
                  else: 
                    cmpt =0
                  
                state = 0

            case 5: #etat 5: deccelerer pour retrouver la ligne
                print("state 5")
                car.acc_state = 2
                car.max_speed_acc = 10
                if car.speed_car <= car.max_speed_acc:
                    car.stop()
                    time.sleep(0.5)
                    state = 6

            case 6: #etat 6: reculer pour reprendre la ligne
                print("state 6")
                if car.line_follower.read_digital() in car.PATTERNS_SLIGHT_LEFT or car.line_follower.read_digital() in car.PATTERNS_SLIGHT_RIGHT or car.line_follower.read_digital() in car.PATTERNS_CENTER:
                    car.stop()
                    time.sleep(0.5)
                    state = 0
                else:
                    car.acc_state = 1
                    car.max_speed_acc = 22
                    car.turn_while_moving((car.last_turn * -1), car.speed_car, "backward")

        
    except KeyboardInterrupt:
        car.stop()
    
    
def stop_test():
  car = Picar()
  car.stop()

def reculons():
      car = Picar()
      car.stop()
      car.speed_car = 22
      car.front_wheels.turn(90)
      while(car.line_follower.read_digital() not in car.PATTERN_STOP):
         # match car.acc_state:
          #  case 0:
           #     car.acceleration(car.max_speed_acc, "forward")
            #case 1:
             #   car.acceleration(car.max_speed_acc, "backward")
            #case 2:
            #    car.decceleration(car.max_speed_acc, "forward")
            #case 3:
             #   car.decceleration(car.max_speed_acc, "backward")
                
          #car.line_following("backward")
          #acc_state = 1
          car.backward(car.speed_car)
          time.sleep(0.1)
          
          
      car.speed_car -=2
      car.backward(car.speed_car)
      time.sleep(0.4)
      car.speed_car -=2
      car.backward(car.speed_car)
      time.sleep(0.4)
      car.stop()
      
    

if __name__ == '__main__':
    try:
      #test()
      reculons()
    finally:
      stop_test()