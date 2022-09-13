'''
https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html
Enable PWM pins
sudo /opt/nvidia/jetson-io/jetson-io.py
->Configure header pins manualy

'''


#Libraries
from shutil import unregister_archive_format
from tempfile import TemporaryFile
import RPi.GPIO as GPIO
import time
import os
from json import loads

def change_range(value, in_min, in_max, out_min, out_max):
    scaled_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return scaled_value

class Drive:
    def __init__(self, servo_gpio, in1_gpio, in2_gpio, ena_gpio, cnf_file="cnf.txt"):
        # path = os.path.dirname(os.path.realpath(__file__))
        # base_path = os.path.abspath(os.path.join(path, os.pardir)) 
        self.cnf_file = cnf_file 
        # print("cnf:", self.cnf_file)
        
        self.in1 = in1_gpio
        self.in2 = in2_gpio
        self.ena = ena_gpio
        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BOARD)
        # set up dc motor
        GPIO.setup(ena_gpio, GPIO.OUT, initial=GPIO.HIGH)
        self.throttle = GPIO.PWM(channel=ena_gpio, frequency_hz=1000)
        self.in1 = in1_gpio
        GPIO.setup(self.in1, GPIO.OUT, initial=GPIO.LOW)
        self.in2 = in2_gpio
        GPIO.setup(self.in2, GPIO.OUT, initial=GPIO.LOW)
        self.throttle.start(0)
        self.throttle.ChangeDutyCycle(0)
        # Set up servo (steering motor)
        # servo thresholds (as %, NOT DUTY!)
        self.left_per = 5.5
        self.right_per = 9.3
        self.forward_per = 7 
        self.read_cnf_file()
        
        GPIO.setup(servo_gpio, GPIO.OUT, initial=GPIO.HIGH)
        self.servo = GPIO.PWM(channel=servo_gpio, frequency_hz=50)
        self.servo.start(self.forward_per)
        self.servo.ChangeDutyCycle(self.forward_per)
        self.steering = 0
        self.speed = 0
        self.stopped = True    
        self.stop()    

    def set_direction(self, forward=True):
        if not forward:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.stopped = True

    def move(self, speed=None):
        self.stopped = False
        if speed is not None:
            self.speed = speed
        if self.speed >= 0:
            self.set_direction(True)
        elif self.speed < 0:
            self.set_direction(False)

        self.throttle.ChangeDutyCycle(abs(speed))

    def set_steering(self, steer=0):
        per = change_range(steer, -100, 100, self.left_per, self.right_per)
        # print("per:", per)
        self.servo.ChangeDutyCycle(per)
        self.steering = steer

    def smooth_steering(self, new_steer=0, max_angle=5):
        angle = new_steer - self.steering
        if abs(angle) > max_angle:
            smooth_steer = int(self.steering + max_angle * (angle/abs(angle)))
        else:
            smooth_steer = new_steer
        # per = change_range(smooth_steer, -100, 100, self.left_duty, self.right_duty)
        self.set_steering(smooth_steer)

    def read_cnf_file(self):
        with open(self.cnf_file, 'r') as f:
            for line in f:
                #print(line)
                dict_line = loads(line)
                if dict_line["data"] == "drive":
                    self.left_per = dict_line['left']
                    self.right_per = dict_line['right']
                    self.forward_per = dict_line['center']
                    self.speed = dict_line['speed']
                break
                
    def write_cnf_file(self, left, right, center, speed):
        replacement = ""
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] == "drive":                    
                    dict_line["left"] = left
                    dict_line["right"] = right
                    dict_line["center"] = center
                    dict_line["speed"] = speed
                replacement = replacement + str(dict_line) + "\n"
        time.sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

    def calibrate_servo(self):
        print("Servo Calibration")
        directions = ["LEFT", "CENTER", "RIGHT"]
        current_per = self.forward_per
        self.set_steering(current_per)
        
        for d in directions:
            save = False
            print("CALIBRATE DIRECTION:", d)
            steering = 0
            while True:
                try:
                    print(f"Current percent for '{d}':", current_per)                    
                    user_input = input("Type new percent ('s' to save value, 'c' to ignore): ")                    
                    if user_input == 'c' or user_input == 'C':
                        save = False
                        break
                    elif user_input == 's' or user_input == 'S':
                        save = True
                        break
                    steering = int(user_input)
                    self.set_steering(steering)
                except:
                    print("invalid input")
            if save:
                if d == 0:
                    self.left_per = steering
                elif d == 1:
                    self.forward_per = steering
                elif d == 2:
                    self.right_per = steering
                self.write_cnf_file(self.left_per, self.right_per, self.forward_per, self.speed)            
            
    def try_speed(self):
        user_input = input("Type speed, 'e' to exit: ")
        while user_input != 'e' and user_input != 'E':
            try:
                speed = int(user_input)
                self.move(speed=speed)
            except:
                print("Invalid input")
            user_input = input("Type speed, 'e' to exit: ")
        self.stop()



def main():
    try:
        drive = Drive(servo_gpio=32, in1_gpio=35, in2_gpio=37, ena_gpio=33)
        while True:
            speed = input("Type speed and then 'y' to continue:")
            while True:
                drive.move(speed=int(speed))
                try:
                    speed = int(input("Type speed and then 'y' to continue:"))
                except ValueError:
                    break
            steer = input("Change steering:")
            while True:
                drive.set_steering(steer=int(steer))
                steer = input("Change steering:")
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
        
if __name__ == "__main__":
    main()
