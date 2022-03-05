from machine import Pin, PWM
import carutils
import time

class Car:
    def __init__(self, pin_throttle, pin_in1, pin_in2, pin_steering, left_steering=20, right_steering=120):
        self.in1 = Pin(pin_in1, Pin.OUT)
        self.in2 = Pin(pin_in2, Pin.OUT)
        self.throttle = PWM(Pin(pin_throttle), frequency=50, duty=0)
        self.steering = PWM(Pin(pin_steering), frequency=50, duty=0)
        self.left_steering = left_steering
        self.right_steering = right_steering
        self.debug = False

    def drive(self, throttle, steering):
        throttle = carutils.map(throttle, -100, 100, -1023, 1023)
        steering = carutils.map(steering, -100, 100, self.left_steering, self.right_steering)
        if throttle > 0:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.in1.value(1)
            self.in2.value(0)
        self.throttle.duty(abs(throttle))
        self.steering.duty(steering)
        if self.debug:
            print("Throttle: " + str(throttle), end='~')
            print("Throttle: " + str(steering))
            
        
    def brake(self):
        self.in1.value(0)
        self.in2.value(0)
        if self.debug:
            print("Stop!")
    
def main():
    PIN_THROTTLE = 12
    PIN_IN1 = 14
    PIN_IN2 = 27
    PIN_STEERING = 33

    car = Car(pin_throttle=PIN_THROTTLE, pin_in1=PIN_IN1, pin_in2=PIN_IN2, pin_steering=PIN_STEERING)
    car.debug = True
    car.drive(throttle=50, steering=0)
    time.sleep(2)
    car.drive(throttle=20, steering=30)
    time.sleep(2)
    car.drive(throttle=70, steering=-50)
    time.sleep(2)
    car.drive(throttle=-100, steering=0)
    time.sleep(2)
    car.drive(throttle=-50, steering=30)
    
if __file__ == '__main__':
    main()
    
    
            
            
        

