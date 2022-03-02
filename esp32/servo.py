from machine import PWM, Pin

# ENA_PIN = 12
# IN1_PIN = 14
# IN2_PIN = 27
# SERVO_PIN = 33


def map_range(var, min_old, max_old, max_new, min_max):
    return int(min_new + var*(max_old - min_old)/(max_new - min_new))

class Car:
    def __innit__(self, ena=12, in1=14, in2=27, servo_pin=33,
                  servo_left=20, servo_right=200):
        self.in1 = Pin(in1, PIN.OUT)
        self.in2 = Pin(in2, PIN.OUT)
        # MIND THE FREQUENCY!!!
        self.ena = PWM(Pin(ena), frequency=50, duty=0)
        self.servo = PWM(Pin(servo_pin), frequency=50, duty=(servo_left + servo_right)//2)
        self.servo_left = servo_left
        self.servo_right = servo_right

    def drive(self, speed, steering):
        '''
        ---parameters
        speed: -100 to 100
        steering: -100 to 100
        '''
        if speed <0:
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.in1.value(0)
            self.in2.value(1)
        ena_duty = map_range(abs(speed), 0, 100, 0, 1023)
        servo_duty = map_range(steering, -100, 100, self.servo_left, self.servo_right)
        self.servo.duty(servo_duty)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)
