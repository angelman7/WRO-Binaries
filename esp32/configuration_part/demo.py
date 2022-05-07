from machine import I2C, Pin, ADC, PWM # to manage ESP32
from time import sleep
PIN_SERVO = 25
forward = 83
right= 95
left = 70
servo = PWM(Pin(PIN_SERVO), freq=50, duty=(right+left)//2)
sleep(1)
while True:
    d = int(input("Duty:"))
    servo.duty(d)
    
    