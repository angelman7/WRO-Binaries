#!/usr/bin/env python

import time
import sys
import signal
import RPi.GPIO as GPIO

import VL53L1X

class Distances:
    def __init__(self, left_xshut=37, left_address=0x41, right_xshut=7, right_address=0x43, default_address=0x29):
        signal.signal(signal.SIGINT, self.exit_handler)
        # set xshut digital output pins
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(left_xshut, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(right_xshut, GPIO.OUT, initial=GPIO.LOW)
        # set left sensor
        GPIO.output(left_xshut, GPIO.HIGH)
        try:

            self.left_tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=default_address)
            self.left_tof.open()
            self.left_tof.change_address(left_address)
            self.left_tof.close()
        except:
            self.left_tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=left_address)
            self.left_tof.open()
            time.sleep(0.2)
            self.left_tof.close()

        
        time.sleep(1)
        # set right sensor
        # GPIO.output(left_xshut, GPIO.LOW)
        GPIO.output(right_xshut, GPIO.HIGH)
        try:

            self.right_tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=default_address)
            self.right_tof.open()
            self.right_tof.change_address(right_address)
            self.right_tof.close()
        except:
            self.right_tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=right_address)
            self.right_tof.open()
            time.sleep(0.2)
            self.right_tof.close()
        # open both sensors
        GPIO.output(left_xshut, GPIO.HIGH)
        GPIO.output(right_xshut, GPIO.HIGH)
        time.sleep(1)
        self.right_tof.open()
        self.left_tof.open()
        self.right_tof.set_timing(66000, 70)
        self.left_tof.set_timing(66000, 70)
        # Start ranging
        # 0 = Unchanged
        # 1 = Short Range
        # 2 = Medium Range
        # 3 = Long Range
        self.left_tof.start_ranging(0)  
        self.right_tof.start_ranging(0)
        self.running = True
        # time.sleep(10)
        

    def left(self):
        return self.left_tof.get_distance()
    
    def right(self):
        return self.right_tof.get_distance()
    
    def exit_handler(self, *args):
        self.running = False
        self.left_tof.stop_ranging()
        self.right_tof.stop_ranging()
        print("Distance stop running!")

    def is_running(self):
        return self.running
        
def main():
    distances = Distances()
    while distances.is_running():
        left_distance = distances.left()
        right_distance = distances.right()
        print("Left Distance: {}mm".format(left_distance))
        print("Right Distance: {}mm".format(right_distance))
        time.sleep(0.05)

if __name__ == "__main__":
    main()