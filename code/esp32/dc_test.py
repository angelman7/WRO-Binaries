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

#  Pins         Value
PIN_SCL =        22
PIN_SDA =        21
PIN_POT =        36
PIN_NEXT_BTN =   2
PIN_PREV_BTN =   5
PIN_HIT_BTN =    0
PIN_IN1 =        14
PIN_IN2 =        12
PIN_ENA =        27
PIN_SERVO =      26
PIN_TRIG_LEFT =  33
PIN_ECHO_LEFT =  32
PIN_TRIG_RIGHT = 19
PIN_ECHO_RIGHT = 18


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
        
dc = DC()
dc.set_direction(True)
while True:
    dc.stop()
        
