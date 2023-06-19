#!/usr/bin/env python3
import tty, sys, termios
from colorsys import rgb_to_hsv
try:
    from modules.grove_i2c_color_sensor_v2 import GroveI2cColorSensorV2
except:
    from grove_i2c_color_sensor_v2 import GroveI2cColorSensorV2
import os
from math import fmod
import time
from json import loads

class Color:
    def __init__(self, filename="cnf.txt"):
        self.sensor = GroveI2cColorSensorV2(0)
        path = os.path.dirname(os.path.realpath(__file__))
        # base_path = os.path.abspath(os.path.join(path, os.pardir)) 
        self.cnf_file = os.path.join(path, filename) 
        
        orange = {"hmin":0, "smin":0, "vmin":0, "hmax":0, "smax":0, "vmax":0}
        blue = {"hmin":0, "smin":0, "vmin":0, "hmax":359, "smax":255, "vmax":255}
        self.lines = {"orange":orange, "blue":blue}
        self.read_cnf_file()
    
    def print(self): 
        # line = self.detect_line()
        # self.ev3.screen.clear()
        # self.ev3.screen.print("Color: " + str(line))
        pass

    def detect_line(self):
        h, s, v = self.read_hsv()
        for color_key in self.lines.keys():
            if self.lines[color_key]['hmin'] <= h <= self.lines[color_key]['hmax'] and \
                self.lines[color_key]['vmin'] <= v <= self.lines[color_key]['vmax']:
            # and 
            # self.lines[color_key]['vmin'] <= v <= self.lines[color_key]['vmax']:
                if color_key == "orange":
                    return 1
                else:
                    return -1
        return 0


    def read_cnf_file(self):
        color_names = self.lines.keys()
        with open(self.cnf_file, 'r') as f:
            for line in f:
                # print(line)
                dict_line = loads(line)
                if dict_line["data"] in color_names:
                    self.lines[dict_line["data"]]['hmin'] = dict_line['hmin']
                    self.lines[dict_line["data"]]['hmax'] = dict_line['hmax']
                    self.lines[dict_line["data"]]['smin'] = dict_line['smin']
                    self.lines[dict_line["data"]]['smax'] = dict_line['smax']
                    self.lines[dict_line["data"]]['vmin'] = dict_line['vmin']
                    self.lines[dict_line["data"]]['vmax'] = dict_line['vmax']

    def write_cnf_file(self, color_name=None):
        color_names = []
        if color_name is None:
            color_names = self.lines.keys()
        else:
            color_names.append(color_name)
        replacement = ""
        # read file to change the values
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] in color_names:
                    dict_line["hmin"] = self.lines[dict_line["data"]]['hmin']
                    dict_line["hmax"] = self.lines[dict_line["data"]]['hmax']
                    dict_line["smin"] = self.lines[dict_line["data"]]['smin']
                    dict_line["smax"] = self.lines[dict_line["data"]]['smax']
                    dict_line["vmin"] = self.lines[dict_line["data"]]['vmin']
                    dict_line["vmax"] = self.lines[dict_line["data"]]['vmax']
                replacement = replacement + str(dict_line) + "\n"
        time.sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

    def read_rgb(self):
        r, g, b, clear = self.sensor.raw
        return r, g, b

    def read_hsv(self):
        r, g, b, clear = self.sensor.raw
        return self.rgb2hsv(r, g, b)

    def rgb2hsv(self, r, g, b):

        #normalize
        (r, g, b) = (r / 255, g / 255, b / 255)
        #convert to hsv
        (h, s, v) = rgb_to_hsv(r, g, b)
        #expand HSV range
        (h, s, v) = (int(h * 179), int(s * 255), int(v * 255))

        return h, s, v

    def calibrate(self):
        try:
            filedescriptors = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
            orange = {"min":[179, 255, 255], "max":[0, 0, 0]}
            blue = {"min":[179, 255, 255], "max":[0, 0, 0]}

            print("Push 's' if sensor is above orange line, 'q' to stop")
            time.sleep(3)
            key = ''
            while key != 'q' or key == 'Q':
                key = sys.stdin.read(1)[0]
                hsv = self.read_hsv()
                print("HSV: ", hsv)
                # print("You pressed", key)
                if key == 's' or key == 'S':
                    if hsv[0] < orange["min"][0]:
                        orange["min"][0] = hsv[0]
                    if hsv[1] < orange["min"][1]:
                        orange["min"][1] = hsv[1]
                    if hsv[2] < orange["min"][2]:
                        orange["min"][2] = hsv[2]
                    if hsv[0] > orange["max"][0]:
                        orange["max"][0] = hsv[0]
                    if hsv[1] > orange["max"][1]:
                        orange["max"][1] = hsv[1]
                    if hsv[2] > orange["max"][2]:
                        orange["max"][2] = hsv[2]
                time.sleep(0.2)
            print("Orange:", orange)
            print("Push 's' if sensor is above blue line, 'q' to stop")
            time.sleep(3)
            key = ''
            while key != 'q' or key == 'Q':
                key = sys.stdin.read(1)[0]
                hsv = self.read_hsv()
                print("HSV: ", hsv)
                if key == 's' or key == 'S':
                    if hsv[0] < blue["min"][0]:
                        blue["min"][0] = hsv[0]
                    if hsv[1] < blue["min"][1]:
                        blue["min"][1] = hsv[1]
                    if hsv[2] < blue["min"][2]:
                        blue["min"][2] = hsv[2]
                    if hsv[0] > blue["max"][0]:
                        blue["max"][0] = hsv[0]
                    if hsv[1] > blue["max"][1]:
                        blue["max"][1] = hsv[1]
                    if hsv[2] > blue["max"][2]:
                        blue["max"][2] = hsv[2]
                time.sleep(0.2)

            print("Orange:", orange)
            print("Blue:", blue)
            time.sleep(2)
            print("Save? (y/n)")
            key = sys.stdin.read(1)[0] 
            if key == 'y' or key == 'Y':
                self.lines = {"orange": {"hmin":orange["min"][0], "smin":orange["min"][1], "vmin":orange["min"][2], "hmax":orange["max"][0], "smax":orange["max"][1], "vmax":orange["max"][2]}, 
                                     "blue": {"hmin":blue["min"][0], "smin":blue["min"][1], "vmin":blue["min"][2], "hmax":blue["max"][0], "smax":blue["max"][1], "vmax":blue["max"][2]}}
                self.write_cnf_file()
                print("New colors: ", self.lines)
            else:
                print("Nothing changed")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)

def main():
    color = Color()
    # color.calibrate()
    while True:
        line = color.detect_line() 
        #if line != 0:
        print(line != 0, " - ", color.read_hsv())

if __name__ == "__main__":
    main()