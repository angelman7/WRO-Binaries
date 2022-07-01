# PILLAR TURN PROGRAM
# can be used for the festival
# DO NOT DELETE

from machine import I2C, Pin, ADC, PWM, UART # to manage ESP32
import ssd1306 # to manage OLED display
from time import sleep, time, time_ns # to manage time
from math import pi, atan2, floor
from ustruct import pack # to write data in sensors' registers
from array import array # to read bytes from sensors' registers
from json import loads# to read files
from hcsr04 import HCSR04
from random import randrange, choice
# from threading import Thread
import _thread
# I2C Color Sensor registers and commands
_CMD_CS      = 0x80
_AUTO_CS     = 0x20

_ENABLE_CS   = 0x00
_ATIME_CS    = 0x01
_WTIME_CS    = 0x03
_AILT_CS     = 0x04
_AIHT_CS     = 0x06
_PERS_CS     = 0x0C
_CONFIG_CS   = 0x0D
_CONTROL_CS  = 0x0F
_ID_CS       = 0x12
_STATUS_CS   = 0x13
_CDATA_CS    = 0x14
_RDATA_CS    = 0x16
_GDATA_CS    = 0x18
_BDATA_CS    = 0x1A

_AIEN_CS     = 0x10
_WEN_CS      = 0x08
_AEN_CS      = 0x02
_PON_CS      = 0x01

_GAINS_CS  = (1, 4, 16, 60)


PIN_SCL = 22
PIN_SDA = 21
PIN_POT = 36
PIN_NEXT_BTN = 2
PIN_PREV_BTN = 5
PIN_HIT_BTN = 0
PIN_IN1 = 12
PIN_IN2 = 14
PIN_ENA = 27
PIN_SERVO = 26
PIN_TRIG_LEFT = 33
PIN_ECHO_LEFT = 32
PIN_TRIG_RIGHT = 19
PIN_ECHO_RIGHT = 18


def map(value, in_min, in_max, out_min, out_max):
    scaled_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return int(scaled_value)

def median(lst):
    n = len(lst)
    s = sorted(lst)
    return (s[n//2-1]/2.0+s[n//2]/2.0, s[n//2])[n % 2] if n else None


class DC:
    def __init__(self):
        self.in1 = Pin(PIN_IN1, Pin.OUT)
        self.in1.value(0)
        self.in2 = Pin(PIN_IN2, Pin.OUT)
        self.in2.value(0)
        self.ena = PWM(Pin(PIN_ENA), freq=50, duty=0)
        self.pot = ADC(Pin(PIN_POT))
        self.pot.atten(ADC.ATTN_11DB)
        self.pot.width(ADC.WIDTH_10BIT)
        self.max_pwm = 1023
        self.forward = True

    def set_direction(self, forward=True):
        if forward:
            self.forward = True
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.forward = False
            self.in1.value(0)
            self.in2.value(1)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)

    def read_pot(self):
        return self.pot.read()

    def move(self, speed=50):
        speed_duty = int((self.max_pwm * speed) / 100)
        self.ena.duty(speed_duty)
        #print(speed_duty)

class Connection:
    def __init__(self, port_no=2, baudrate=115200, num_of_vals=2, digits_per_val=4):
        try:
            self.ser = UART(port_no, baudrate)
            print("Connected")
        except:
            print("Fail to connect")
        sleep(1)
        self.num_of_vals = num_of_vals
        self.digits_per_val = digits_per_val   

    def send_data(self, data, open_sign="$"):
        myString = open_sign
        for d in data:
            myString += str(d) + "#"
        myString = myString[:-1] # remove last '#'
        myString += "\n"
        try:
            self.ser.write(myString.encode())
            #print("Send:", myString)
        except:
            print("Data Transmission Failed ")

    def test(self):
        while True:
            if self.ser.any():
                str_data = self.ser.readline()
                print("Read", str_data.decode())
                self.ser.write(str_data)
                print("Send:", str_data.decode(), "\n")
            else:
                print("No message\n")

    def get_data(self, open_sign="$"):
        if self.ser.any():
            str_data = self.ser.readline().decode()
            #print(str_data)
            if str_data[0] == open_sign:
                str_data = str_data[1:-1] # remove '$' and '\n'
                data = str_data.split('#')
                data_list = []
                for d in data:
                    #if d.isdigit():
                        try:
                            i = int(d)
                            data_list.append(int(d))
                        except ValueError:
                            print("Value Error in Incoming Data")
                            return None
                    #else:
                      #  return None
                return data_list 
        return None
    
class Servo:
    def __init__(self):
        self.cnf_file = "cnf.txt"
        self.left_duty = 0
        self.right_duty = 0
        self.forward_duty = 0 
        self.read_cnf_file()
        self.servo = PWM(Pin(PIN_SERVO), freq=50, duty=self.forward_duty)

    def steering(self, steer=0):
        duty = map(steer, -100, 100, self.left_duty, self.right_duty)
        #print("Duty: ", duty)
        self.servo.duty(duty)

    def set_duty(self, duty):
        self.servo.duty(duty)

    def read_cnf_file(self):
        with open(self.cnf_file, 'a') as f:
            for line in f:
                #print(line)
                dict_line = loads(line)
                if dict_line["data"] == "servo":
                    self.left_duty = dict_line["left"]
                    self.right_duty = dict_line["right"]
                    self.forward_duty = dict_line["forward"]
                    break
# DC MOTOR TRIAL
# dc = DC()
# dc.stop()
# dc.set_direction()
# dc.move(speed=map(dc.read_pot(), 0, 1023, 0, 100))
# sleep(2)
# dc.stop()

# CONNECTION WITH JETSON TRIAL
connection = Connection()
servo = Servo()
dc = DC()

while True:
    connection.send_data(data="!0000#0000#0000#0000" ,open_sign = "!")
    datalist = connection.get_data(open_sign="$") #datalist[0] -> servo steering,
                                                  #datalist[1] -> dc speed
    if datalist is not None:
        print(datalist)
        break

dc_speed = int(datalist[1])
dc_steering = int(datalist[0])
dc.stop()
servo.steering(60)
dc.set_direction()
dc.move(speed=dc_speed)
sleep(1.6)
servo.steering(dc_steering)
sleep(1)
dc.stop()

# DRAFT VALUES v.2
# servo.steering(40)
# sleep(1)
# servo.steering(56)
# sleep(1)
# servo.steering(70)


# DRAFT VALUES
# servo.steering(-90)
# sleep(1)
# servo.steering(20)
# sleep(1)
# servo.steering(150)


