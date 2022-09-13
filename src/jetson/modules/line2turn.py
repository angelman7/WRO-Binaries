try:
    from modules.color import Color
except:
    from color import Color
from json import loads
import time
import os

class Line2Turn:
    def __init__(self, cnf_file="cnf.txt"):
        self.sensor = Color()
        self.colors = {"blue_line_sensor":[(0, 0, 0), (179, 255, 255)], "orange_line_sensor":[(0, 0, 0), (179, 255, 255)]}
        self.cnf_file = cnf_file
        self.read_cnf_file()

    def detect_line(self):
        h, s, v = self.sensor.read_hsv()
        for line_name, hsv_thresholds in self.colors.items():
            if hsv_thresholds[0][0] <= h <= hsv_thresholds[1][0] and \
                hsv_thresholds[0][1] <= s <= hsv_thresholds[1][1] and \
                hsv_thresholds[0][2] <= v <= hsv_thresholds[1][2]:
                if line_name == "blue_line_sensor":
                    return -1
                elif line_name == "orange_line_sensor":
                    return 1
        return 0


    def read_cnf_file(self):
        with open(self.cnf_file, 'r') as f:
            for line in f:
                print(line)
                dict_line = loads(line)
                for line_name in self.colors.keys():
                    if dict_line["data"] == line_name:
                        self.colors[line_name][0] = dict_line['hsv_lower']
                        self.colors[line_name][1] = dict_line['hsv_upper']
                                        
    def write_cnf_file(self, new_colors):
        replacement = ""
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                for line_name in self.colors.keys():
                    if dict_line["data"] == line_name:                    
                        dict_line["hsv_lower"] = new_colors[line_name][0]
                        dict_line["hsv_upper"] = new_colors[line_name][1]
                    replacement = replacement + str(dict_line) + "\n"
        time.sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

def main():
    l = Line2Turn()
    while True:
        h, s, v = l.sensor.read_hsv()
        print(l.colors["blue_line_sensor"])
        print(l.colors["orange_line_sensor"])
        print(h, s, v)
        for line_name, hsv_thresholds in l.colors.items():
            if hsv_thresholds[0][0] <= h <= hsv_thresholds[1][0] and \
                hsv_thresholds[0][1] <= s <= hsv_thresholds[1][1] and \
                hsv_thresholds[0][2] <= v <= hsv_thresholds[1][2]:
                if line_name == "blue_line_sensor":
                    print(-1)
                elif line_name == "orange_line_sensor":
                    print(1)
            else:
                print(0)

if __name__ == "__main__":
    main()