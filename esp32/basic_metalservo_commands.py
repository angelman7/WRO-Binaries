#https://docs.micropython.org/en/latest/library/machine.PWM.html
# duty_ut16 becomes duty PLAIN

from machine import PWM, Pin, freq
from time import sleep

pwm = PWM(Pin(25))

# set freq to 50, duty to 0 to initialise and stop previous movement of servo
pwm.freq(50) # ALWAYS 50
pwm.duty(0)

for i in range(1, 4):
    pwm.freq(50)
    pwm.duty(10)
    sleep(1)

pwm.duty(0)