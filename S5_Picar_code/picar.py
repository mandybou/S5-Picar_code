from . import front_wheels
from . import back_wheels

class Picar():
    def __init__(self):
        self.front_wheels = front_wheels.Front_Wheels()
        self.back_wheels = back_wheels.Back_Wheels()
    
    def forward(self, speed):
        self.back_wheels.forward()
        self.back_wheels.speed = speed
    def backward(self, speed):
        self.back_wheels.backward()
        self.back_wheels.speed = speed
    
    def turn_while_moving(self, angle, speed, direction):
        if direction == "forward":
            self.front_wheels.turn(angle)
            self.forward(speed)
        elif direction == "backward":
            self.front_wheels.turn(angle)
            self.backward(speed)

def test():
    import time
    car = Picar()
    try:
        while True:
            print("forward")
            car.forward(30)
            time.sleep(2)
            print("backward")
            car.backward(30)
            time.sleep(2)
            print("turn left")
            car.turn_while_moving(-90, 30, "forward")
            time.sleep(2)
            print("turn right")
            car.turn_while_moving(90, 30, "forward")
            time.sleep(2)
            print("turn left")
            car.turn_while_moving(-90, 30, "backward")
            time.sleep(2)
            print("turn right")
            car.turn_while_moving(90, 30, "backward")
            time.sleep(2)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    test()