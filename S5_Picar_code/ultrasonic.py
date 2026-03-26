import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)

TRIG = 12
ECHO = 16
i=0

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

GPIO.output(TRIG, False)
print("Calibration.....")
time.sleep(2)

print ("Placez un objet......")


try:
    while True:
       GPIO.output(TRIG, True)
       time.sleep(0.00001)
       GPIO.output(TRIG, False)

       while GPIO.input(ECHO)==0:
          pulse_start = time.time()

       while GPIO.input(ECHO)==1:
          pulse_end = time.time()

       pulse_duration = pulse_end - pulse_start

#       distance = pulse_duration * 17150
       distance = pulse_duration * 16666

       distance = round(distance, 2)
  
       if distance<=40 and distance>=3:
          print ("distance:",distance,"cm")
          i=1
          
       if distance>40 and i==1:
          print ("placez un objet....")
          i=0
       time.sleep(0.06)

except KeyboardInterrupt:
     GPIO.cleanup()