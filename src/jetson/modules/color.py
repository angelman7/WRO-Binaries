from colorsys import rgb_to_hsv
try:
    from modules.grove_i2c_color_sensor_v2 import GroveI2cColorSensorV2
except:
    from grove_i2c_color_sensor_v2 import GroveI2cColorSensorV2

from math import fmod
from time import sleep
from json import loads

class Color:
    def __init__(self, filename="cnf.txt"):
        self.sensor = GroveI2cColorSensorV2()
        self.cnf_file = filename
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
                self.lines[color_key]['smin'] <= s <= self.lines[color_key]['smax'] and \
                self.lines[color_key]['vmin'] <= v <= self.lines[color_key]['vmax']:
                if color_key == "orange":
                    return 1
                else:
                    return -1
        return 0

    # def hsv(self):
    #     r, g, b = self.sensor.rgb()
    #     r, g, b = r / 100, g / 100, b / 100
    #     maxc = max(r, g, b)
    #     minc = min(r, g, b)
    #     v = maxc
    #     if minc == maxc:
    #         return 0.0, 0.0, v
    #     s = (maxc-minc) / maxc
    #     rc = (maxc-r) / (maxc-minc)
    #     gc = (maxc-g) / (maxc-minc)
    #     bc = (maxc-b) / (maxc-minc)
    #     if r == maxc:
    #         h = bc-gc
    #     elif g == maxc:
    #         h = 2.0+rc-bc
    #     else:
    #         h = 4.0+gc-rc
    #     h = (h/6.0) % 1.0
    #     return min(359, int(h*359)), min(255, int(s*255)), min(255, int(v*255))

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
        sleep(0.1)
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

    
def main():
    color = Color()
    while True:
        print(color.detect_line())

if __name__ == "__main__":
    main()