import RPi.GPIO as GPIO
import time

# min 3 , max 13
LEFT_DUTY = 3
RIGHT_DUTY = 11
MAX_SMOOTH_STEER = 5

def _map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Steering:
	def __init__(self, servo_pin=33):
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(servo_pin, GPIO.OUT, initial=GPIO.HIGH)
		self.servo = GPIO.PWM(channel=servo_pin, frequency_hz=50)
		self.servo.start((LEFT_DUTY + RIGHT_DUTY) / 2)
		self.steer = 0

	def set_steering(self, steer=0):
		duty = _map(steer, -100, 100, LEFT_DUTY, RIGHT_DUTY)
		# print(f'Servo duty: {duty}', end='\t')
		self.servo.ChangeDutyCycle(duty)
		self.steer = steer			

	def smooth_steering(self, new_steer=0, max_angle=MAX_SMOOTH_STEER):
		angle = new_steer - self.steer
		if abs(angle) > max_angle:
			smooth_steer = int(self.steer + max_angle * (angle/abs(angle)))
		else:
			smooth_steer = new_steer
		
		try:
			smooth_steer = smooth_steer/abs(smooth_steer) * max(10, abs(smooth_steer))
			self.set_steering(smooth_steer)
		except ZeroDivisionError:
			pass	
		return self.steer


def main():
	steering = Steering()
	while True:
		steer = int(input("Steering (-100 to 100, 200 to exit)"))
		if steer == 200:
			steering.set_steering(0)
			break
		steering.set_steering(steer)
		
if __name__ == "__main__":
	main()