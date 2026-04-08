import front_wheels
import back_wheels
import time
from SunFounder_Line_Follower import Line_Follower
import ultrasonic


class Picar():
    max_speed = 25
    max_speed_acc = 25

    #0: acceleration avant, 1: acceleration arriere, 2: decceleration avant, 3: decceleration arriere
    acc_state = 0
    speed_car = 15
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
        
    def acceleration(self, speed, direction):
        if(self.speed_car < speed):
            self.speed_car += 2
            if direction == "forward":
                self.forward(self.speed_car)
            elif direction == "backward":
                self.backward(self.speed_car)
            
    def decceleration(self, speed, direction):
        if(self.speed_car > speed):
            self.speed_car -= 2
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
            if direction == "forward":
              self.acc_state = 0
              self.turn_while_moving(0, self.speed_car, "forward")
            else:
              self.acc_state = 1
              self.turn_while_moving(0, self.speed_car, "backward")

        elif status in self.PATTERNS_SLIGHT_LEFT:
            print("slight left")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed
            angle = -15 if direction == "forward" else 15
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_HARD_LEFT:
            print("hard left")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed - 5
            if direction == "forward":
                angle = -25
                self.acc_state = 0
            else:
                angle = 40
                self.acc_state = 1
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_SLIGHT_RIGHT:
            print("slight right")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed
            if direction == "forward":
                angle = 15
                self.acc_state = 0
            else:
                angle = -15
                self.acc_state = 1
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERNS_HARD_RIGHT:
            print("hard right")
            self.lost_counter = 0
            self.max_speed_acc = self.max_speed - 5
            if direction == "forward":
                angle = 25
                self.acc_state = 0
            else:
                angle = -40
                self.acc_state = 1
            self.turn_while_moving(angle, self.speed_car, direction)
            self.last_turn = angle

        elif status in self.PATTERN_LOST:
            print(self.lost_counter)
            print("lost")
            if self.lost_counter >= 1:
                self.lost_counter = 0
                return 1
              #recovery_direction = "backward" if direction == "forward" else "forward"
              #self.decceleration(17)
              #self.stop()
              #self.forward(0)
              
              #time.sleep(0.4)
              #cmpt = 0
              #print(recovery_direction)
              #while(self.line_follower.read_digital() in self.PATTERN_LOST):
                #cmpt+
                #if self.speed_car <= 25:
                  #self.speed_car +=2
                #self.turn_while_moving((self.last_turn * -1), self.speed_car, "backward")
                #time.sleep(0.4)
                #return 1              
              #self.stop()
              #self.front_wheels.turn(90 + self.last_turn)
              #self.acceleration(25)
              #time.sleep(0.1)
              #self.lost_counter = 0
              #self.speed_car = self.max_speed
            else:
              #self.turn_while_moving((self.last_turn), self.max_speed - 7, direction)
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
        time.sleep(0.2)
        print(car.speed_car)

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
                while((time.time() - start) <= 2.4):
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
                while(cmpt < 1):
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
                car.max_speed_acc = 19
                if car.speed_car <= car.max_speed_acc:
                    car.stop()
                    time.sleep(0.5)
                    state = 6

            case 6: #etat 6: revuler pour reprendre la ligne
                print("state 6")
                if car.line_follower.read_digital() not in car.PATTERN_LOST:
                    car.stop()
                    time.sleep(0.5)
                    state = 0
                else:
                    car.acc_state = 1
                    car.max_speed_acc = 20
                    car.turn_while_moving((car.last_turn * -1), car.speed_car, "backward")

                
                
              
                    

        #etat 4: Evitement d'obstacle
        
    except KeyboardInterrupt:
        car.stop()
    
    
def stop_test():
  car = Picar()
  car.stop()



if __name__ == '__main__':
    try:
      test()
    finally:
      stop_test()