import RPi.GPIO as GPIO
import time
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


fc = 2
fe = 16.7
fs = 8
Rp = 0.2     #  (dB)
Rs = 60      #  (dB)

Wp = fc / (fe/2)
Ws = fs / (fe/2)


t_vals = []
raw_vals = []
filtre_vals = []
cheb1_vals = []


N_cheb1, Wn_cheb1 = signal.cheb1ord(Wp, Ws, Rp, Rs)
print("Ordre Chebyshev I:", N_cheb1)

d, c = signal.cheby1(N_cheb1, Rp, Wn_cheb1, btype="low")
zi_cheb1 = signal.lfilter_zi(d, c)

def filtre_pb_cheb1(x):
    global zi_cheb1
    y, zi_cheb1 = signal.lfilter(d, c, [x], zi=zi_cheb1)
    return y[0]

last_valid_distance = None
INIT_SAMPLES = 5       # nombre d'echantillons pour stabiliser
SPIKE_THRESHOLD = 0.20 # 20 %

GPIO.setmode(GPIO.BCM)

TRIG = 12
ECHO = 16
i = 0

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
print("Calibration.....")
time.sleep(2)

print("Placez un objet......")

try:
    while True:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 16666
        distance = round(distance, 2)

        # bruit artificiel pour test
        if ((len(t_vals) % 100) == 0) and (len(t_vals) != 0):
            distance -= 25
            
        distance_filtre_cheb1 = filtre_pb_cheb1(distance)
        distance_filtre_cheb1 = round(distance_filtre_cheb1, 2)

        if 3 <= distance <= 40:

            # ===== PHASE 1 : INITIALISATION =====
            if len(filtre_vals) < INIT_SAMPLES:
                distance_filtre = distance
                last_valid_distance = distance

            # ===== PHASE 2 : DETECTION DE SPIKE =====
            else:
                variation = abs(distance - last_valid_distance) / last_valid_distance

                if variation > SPIKE_THRESHOLD:
                    print("Spike supprime :", distance)
                    distance_filtre = last_valid_distance
                else:
                    distance_filtre = distance
                    last_valid_distance = distance

            print("distance sans filtre:", distance)
            print("distance filtree:", distance_filtre)

            t_vals.append(len(t_vals))
            raw_vals.append(distance)
            filtre_vals.append(distance_filtre)
            cheb1_vals.append(distance_filtre_cheb1)
            i = 1

        if distance > 40 and i == 1:
            print("placez un objet....")
            i = 0

        time.sleep(0.06)

        # ===== PLOT =====
        if len(t_vals) == 200:
            plt.figure()
            plt.plot(t_vals, raw_vals, label="Sans filtre")
            plt.plot(t_vals, filtre_vals, label="Filtre anti-spike")
            plt.plot(t_vals, cheb1_vals, label="Filtre cheb1")
            plt.xlabel("Echantillon")
            plt.ylabel("Distance (cm)")
            plt.title("Comparaison filtre anti-spike")
            plt.legend()
            plt.savefig("graph_peak.png")
            print("Graph sauvegarde en graph_peak_aber.png")

except KeyboardInterrupt:
    GPIO.cleanup()