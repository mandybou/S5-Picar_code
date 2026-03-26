import front_wheels
import back_wheels
import time
from SunFounder_Line_Follower import Line_Follower

class Picar():

    max_speed = 70
    speed_car = 0
    
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
            
    def line_follower(self):
        lf = Line_Follower.Line_Follower()
        
        status = lf.read_digital()
        print(status)
        if status in self.PATTERNS_CENTER:
            print("center")
            self.forward(20)
            self.back_wheels.speed = self.speed_car
            
        elif status in self.PATTERNS_SLIGHT_LEFT:
            print("sligth left")
            self.turn_while_moving(-15, self.speed_car, "forward")
            
        elif status in self.PATTERNS_HARD_LEFT:
            print("left")
            self.turn_while_moving(-25, self.speed_car, "forward")
            
        elif status in self.PATTERNS_SLIGHT_RIGHT:
            print("sligth rigth")
            self.turn_while_moving(15, self.speed_car, "forward")
            
        elif status in self.PATTERNS_HARD_RIGHT:
            print("rigth")
            self.turn_while_moving(25, self.speed_car, "forward")
            
        elif status in self.PATTERN_LOST:
            self.backward(20)
            #self.stop()

def test():
    car = Picar()
    #car.turn_while_moving(30,40,"forward")
    #time.sleep(5)
    #car.turn_while_moving(-30,40,"forward")
    #time.sleep(5)
    
    #car.back_wheels.speed = 40
    car.forward(30)
    #time.sleep(10)
    #lf = Line_Follower.Line_Follower()
    #while True:
    
      #status = lf.read_digital()
      #print(status)
    while True:
      car.line_follower()
      time.sleep(0.2)
    
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
    test()
    #stop_test()