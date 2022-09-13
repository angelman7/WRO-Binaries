'''
https://github.com/NVIDIA/jetson-gpio
To read the value of a channel, use:

GPIO.input(channel)
This will return either GPIO.LOW or GPIO.HIGH.

We need PULL UP resistor (1K)
'''

#Libraries
import RPi.GPIO as GPIO
import time

class Button:
    def __init__(self, gpio_pin):
        self.gpio = gpio_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio, GPIO.IN)  # button pin set as input

    def is_pressed(self):
        return GPIO.input(self.gpio) == GPIO.LOW

    def wait(self):
        GPIO.wait_for_edge(self.gpio, GPIO.FALLING)

def main():
    try:
        counter = 0
        up = Button(12)
        center = Button(16)
        down = Button(18)
        while True:
            if up.is_pressed():
                print("UP pressed!", counter)
                counter += 1
                time.sleep(0.5)
            if center.is_pressed():
                print("Center pressed!", counter)
                counter += 1
                time.sleep(0.5)
            if down.is_pressed():
                print("DOWN pressed!", counter)
                counter += 1
                time.sleep(0.5)
                
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()

if __name__ == "__main__":
    main()
