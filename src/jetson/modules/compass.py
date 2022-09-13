import os
import time
try:
    from modules.bmm150 import *
except:
    from bmm150 import *
from json import loads

class Compass:
    def __init__(self, cnf_file="cnf.txt"):
        self.cnf_file = cnf_file
        self.bias ={'x': 4.24607858358263, 'y': -8.861616697805175, 'z': -13.35804206281705}
        self.scale = {'x': 0.9949934729571662, 'y': 0.9995958202425628, 'z': 1.0054657739464354}
        self.orientations = None
        self.orientations_1 = [0, 90, 180, 270]
        self.orientations_2 = [0, 90, 180, 270]
        self.read_cnf_file()
        print(self.bias, self.scale)
        I2C_BUS = 0x01   #default use I2C1
        # I2C address select, that CS and SDO pin select 1 or 0 indicates the high or low level respectively. There are 4 combinations: 
        ADDRESS_3 = 0x13   # (CSB:1 SDO:1) default i2c address
        self.bmm150 = DFRobot_bmm150_I2C(I2C_BUS, ADDRESS_3)
        while self.bmm150.ERROR == self.bmm150.sensor_init():
            print("sensor init error, please check connect") 
            time.sleep(1)

        '''
            Set sensor operation mode
            opMode:
                POWERMODE_NORMAL  Get geomagnetic data normally
                POWERMODE_FORCED  Single measurement, the sensor restores to sleep mode when the measurement is done.
                POWERMODE_SLEEP   Users can visit all the registers, but can't measure geomagnetic data
                POWERMODE_SUSPEND At the time the sensor cpu doesn't work and can't implement any operation. Users can only visit the content of the control register BMM150_REG_POWER_CONTROL
        '''
        self.bmm150.set_operation_mode(self.bmm150.POWERMODE_NORMAL)

        '''
            Set preset mode, make it easier for users to configure sensor to get geomagnetic data
            presetMode:
                PRESETMODE_LOWPOWER       Low power mode, get a small number of data and take the mean value.
                PRESETMODE_REGULAR        Regular mode, get a number of data and take the mean value.
                PRESETMODE_ENHANCED       Enhanced mode, get a large number of data and take the mean value.
                PRESETMODE_HIGHACCURACY   High accuracy mode, get a huge number of data and take the mean value.
        '''
        self.bmm150.set_preset_mode(self.bmm150.PRESETMODE_HIGHACCURACY)

        '''
            Set the rate of obtaining geomagnetic data, the higher, the faster (without delay function)
            rate:
                RATE_02HZ
                RATE_06HZ
                RATE_08HZ
                RATE_10HZ        #(default rate)
                RATE_15HZ
                RATE_20HZ
                RATE_25HZ
                RATE_30HZ
        '''
        self.bmm150.set_rate(self.bmm150.RATE_15HZ)
        
        '''
            Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required. When disabled, the geomagnetic data at x, y, and z will be inaccurate.
            Refer to readme file if you want to configure more parameters.
        '''
        self.bmm150.set_measurement_xyz()

    def init_orientation(self):
        index = 0
        or1_index = 0
        heading = self.heading()
        for ort in self.orientations_1:
            ranges = []
            min_value = (359 + ort - 15) % 360
            max_value = (ort + 15) % 360
            if min_value < max_value:
                ranges.append((min_value, max_value))
            elif min_value > max_value:
                ranges.append((min_value, 360))
                ranges.append((0, max_value))                
            for r in ranges:
                if r[0] < heading < r[1]:                
                    or1_index = index
                    break
            index += 1
        index = 0
        or2_index = 0
        for ort in self.orientations_2:
            ranges = []
            min_value = (359 + ort - 15) % 360
            max_value = (ort + 15) % 360
            if min_value < max_value:
                ranges.append((min_value, max_value))
            elif min_value > max_value:
                ranges.append((min_value, 360))
                ranges.append((0, max_value))                
            for r in ranges:
                if r[0] < heading < r[1]:                  
                    or2_index = index
                    break
            index += 1
        or1_error = self.orientations_1[or1_index] - heading
        or2_error = self.orientations_2[or2_index] - heading
        if abs(or2_error) < abs(or1_error):
            self.orientations = self.orientations_2
            self.current_target_index = or2_index
        else:
            self.orientations = self.orientations_1
            self.current_target_index = or1_index

    def turn(self, direction=1):
        self.current_target_index = (self.current_target_index + direction) % 4

    def heading(self):
        # return self.bmm150.get_compass_degree()
        geomagnetic = self.bmm150.get_geomagnetic()
        x = geomagnetic[0]
        y = geomagnetic[1]
        z = geomagnetic[2]
        x, y, z = self.fix_xyz(x, y, z)
        bearing  = math.atan2(y, x) 
        if (bearing < 0):
            bearing += 2 * math.pi
        return int(math.degrees(bearing))

    def read_xyz(self):
        return self.bmm150.get_geomagnetic()

    def fix_xyz(self, x, y, z):
        x = self.scale["x"] * (x - self.bias['x']) 
        y = self.scale["y"] * (y - self.bias['y'])
        z = self.scale["z"] * (z - self.bias['z'])
        return x, y, z

    # def test_compass(self):
    #     geomagnetic = self.bmm150.get_geomagnetic()
    #     x = geomagnetic[0]
    #     y = geomagnetic[1]
    #     z = geomagnetic[2]
    #     print("*****TEST COMPASS*****")
    #     print(f"Original x {x}")
    #     print(f"Original y {y}")
    #     print(f"Original z {z}")
    #     x, y, z = self.fix_xyz(x, y, z)
    #     or_degree = self.bmm150.get_compass_degree()
    #     my_degree = self.heading(geomagnetic[0], geomagnetic[1])
    #     cal_degrees = self.heading(x, y)
    #     print(f"Original heading {or_degree}\nmy heading {my_degree}\ncalibrated heading {cal_degrees}") 
    #     print("************************")


    def calibrate(self, seconds=60):
        print("Starting calibration")
        starting = time.time()
        min_values = [1000, 1000, 1000]
        max_values = [-1000, -1000, -1000]
        while time.time() - starting < seconds:
            geomagnetic = self.bmm150.get_geomagnetic()
            if min_values[0] > geomagnetic[0]:
                min_values[0] = geomagnetic[0]
            if max_values[0] < geomagnetic[0]:
                max_values[0] = geomagnetic[0]
            if min_values[1] > geomagnetic[1]:
                min_values[1] = geomagnetic[1]
            if max_values[1] < geomagnetic[1]:
                max_values[1] = geomagnetic[1]
            if min_values[2] > geomagnetic[2]:
                min_values[2] = geomagnetic[2]
            if max_values[2] < geomagnetic[2]:
                max_values[2] = geomagnetic[2]
        # print("Min:", min_values)
        # print("Max:", max_values)
        
        self.bias['x'] = (max_values[0] + min_values[0]) / 2
        self.bias['y'] = (max_values[1] + min_values[1]) / 2
        self.bias['z'] = (max_values[2] + min_values[2]) / 2

        delta_axis = [(max_values[0] - min_values[0]) / 2,
                    (max_values[1] - min_values[1]) / 2,
                    (max_values[2] - min_values[2]) / 2]

        delta_avg = sum(delta_axis) / 3

        self.scale['x'] = delta_avg / delta_axis[0]
        self.scale['y'] = delta_avg / delta_axis[1]
        self.scale['z'] = delta_avg / delta_axis[2]
        
        # print("Bias:", self.bias)
        # print("Scale:", self.scale)     


    
    def read_cnf_file(self):
        with open(self.cnf_file, 'r') as f:
            for line in f:
                print(line)
                dict_line = loads(line)
                if dict_line["data"] == "compass_bias":
                    self.bias["x"] = dict_line["x"]
                    self.bias["y"] = dict_line["y"]
                    self.bias["z"] = dict_line["z"]
                if dict_line["data"] == "compass_scale":
                    self.scale["x"] = dict_line["x"]
                    self.scale["y"] = dict_line["y"]
                    self.scale["z"] = dict_line["z"]
                if dict_line["data"] == "compass_1":
                    self.orientations_1[0] = dict_line['a']
                    self.orientations_1[1] = dict_line['b']
                    self.orientations_1[2] = dict_line['c']
                    self.orientations_1[3] = dict_line['d']
                if dict_line["data"] == "compass_2":
                    self.orientations_2[0] = dict_line['a']
                    self.orientations_2[1] = dict_line['b']
                    self.orientations_2[2] = dict_line['c']
                    self.orientations_2[3] = dict_line['d']
                                        
    def write_cnf_file(self, new_bias=None, new_scale=None):
        if new_bias is None:
            new_bias = self.bias
        if new_scale is None:
            new_scale = self.scale
        replacement = ""
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] == "compass_bias":                    
                    dict_line["x"] = new_bias["x"]
                    dict_line["y"] = new_bias["y"]
                    dict_line["z"] = new_bias["z"]  
                if dict_line["data"] == "compass_scale":                    
                    dict_line["x"] = new_scale["x"]
                    dict_line["y"] = new_scale["y"]
                    dict_line["z"] = new_scale["z"]
                if dict_line["data"] == "compass_1":
                    dict_line["a"] = self.orientations_1[0]
                    dict_line["b"] = self.orientations_1[1]
                    dict_line["c"] = self.orientations_1[2]
                    dict_line["d"] = self.orientations_1[3]
                if dict_line["data"] == "compass_2":
                    dict_line["a"] = self.orientations_2[0]
                    dict_line["b"] = self.orientations_2[1]
                    dict_line["c"] = self.orientations_2[2]
                    dict_line["d"] = self.orientations_2[3]              
                replacement = replacement + str(dict_line) + "\n"
        time.sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

                            
def main():
    compass = Compass()
    try:
        while True:
            # compass.test_compass()
            print(compass.heading())
            time.sleep(3)
    except KeyboardInterrupt:
            print("Measurements stopped by keyboard")
    # print("wait")
    # time.sleep(10)
    # print("start")
    # compass.calibrate(60)
    # print("stop")
    # time.sleep(30)
    # print("print")
    
if __name__ == "__main__":
    main()