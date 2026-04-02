import smbus2 as smbus
import math
import time

class Line_Follower(object):
	def __init__(self, address=0x11, references=None):
		self.bus = smbus.SMBus(1)
		self.address = address
		self._references = references if references is not None else [150, 150, 150, 150, 150]
		self._white_values = [0, 0, 0, 0, 0]
		self._black_values = [0, 0, 0, 0, 0]

	def read_raw(self):
		for i in range(0, 5):
			try:
				raw_result = self.bus.read_i2c_block_data(self.address, 0, 10)
				Connection_OK = True
				break
			except:
				Connection_OK = False

		if Connection_OK:
			return raw_result
		else:
			return False

	def read_analog(self, trys=5):
		for _ in range(trys):
			raw_result = self.read_raw()
			if raw_result:
				analog_result = [0, 0, 0, 0, 0]
				for i in range(0, 5):
					high_byte = raw_result[i*2] << 8
					low_byte = raw_result[i*2+1]
					analog_result[i] = high_byte + low_byte
				return analog_result
		raise IOError("Line follower read error. Please check the wiring.")

	def read_digital(self):
		lt = self.read_analog()
		digital_list = []

		for i in range(5):
			# Si la ligne noire donne des valeurs plus petites que le fond blanc
			if self._black_values[i] < self._white_values[i]:
				digital_list.append(1 if lt[i] < self._references[i] else 0)
			# Si jamais c'est l'inverse sur ton capteur
			else:
				digital_list.append(1 if lt[i] > self._references[i] else 0)

		return digital_list

	def get_average(self, mount):
		if not isinstance(mount, int):
			raise ValueError("Mount must be integer")
		average = [0, 0, 0, 0, 0]
		lt_list = [[], [], [], [], []]
		for _ in range(mount):
			lt = self.read_analog()
			for lt_id in range(5):
				lt_list[lt_id].append(lt[lt_id])
		for lt_id in range(5):
			average[lt_id] = int(math.fsum(lt_list[lt_id]) / mount)
		return average

	def calibrate(self, samples=50):
		print("Calibration : placez les capteurs sur le FOND CLAIR.")
		time.sleep(2)
		self._white_values = self.get_average(samples)
		print(f"Fond mesuré : {self._white_values}")

		print("Calibration : placez les capteurs sur la LIGNE FONCÉE.")
		time.sleep(2)
		self._black_values = self.get_average(samples)
		print(f"Ligne mesurée : {self._black_values}")

		self._references = [
			int((self._white_values[i] + self._black_values[i]) / 2)
			for i in range(5)
		]

		print(f"Calibration terminée. Références : {self._references}")

	@property
	def references(self):
		return self._references

	@references.setter
	def references(self, value):
		self._references = value