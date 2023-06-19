import RPi.GPIO as GPIO
import time

def run_method(channel, test=1):
    print("test" + str(test))

class Button:
    def __init__(self, pin, method_to_run=None):
        self.method_to_run = method_to_run
        # GPIO.cleanup()
        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(pin, GPIO.FALLING, callback=self.method_to_run, bouncetime=1000)
        print(self.method_to_run)
        self.last_run_time = time.time()
        self.pin = pin
        

    def set_method(self, method):
        self.method_to_run = method
        current_time = time.time()
        if current_time - self.last_run_time > 1:
            GPIO.cleanup()
            GPIO.setmode(GPIO.BOARD) 
            GPIO.setup(self.pin, GPIO.IN)
            GPIO.add_event_detect(self.pin, GPIO.FALLING, callback=self.method_to_run, bouncetime=1000)
            self.last_run_time = current_time

def demo1(channel):
    print("Button pressed - demo1!") 

def demo2(channel):
    print("Button pressed - demo2!")


def main():
    print("Starting BUTTON demo now! Press CTRL+C to exit")
    button = Button(21, demo1)
    print("PRESS BUTTON to turn on or off to try button! Press CTRL+C to exit")
    try:
        start_time = time.time()
        while True:                                    
            if time.time() - start_time > 10:
                print("change method")
                button.set_method(demo2)
                start_time = time.time()
            
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()


    