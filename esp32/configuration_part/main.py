'''
MAIN CONFIGURATION FILE
'''
from machine import I2C, Pin, ADC, PWM # to manage ESP32
import ssd1306 # to manage OLED display
from time import sleep, time # to manage time
from math import pi, atan2, floor 
from ustruct import pack # to write data in sensors' registers
from array import array # to read bytes from sensors' registers
from json import loads# to read files
from hcsr04 import HCSR04

# I2C Color Sensor registers and commands
_CMD_CS      = 0x80
_AUTO_CS     = 0x20

_ENABLE_CS   = 0x00
_ATIME_CS    = 0x01
_WTIME_CS    = 0x03
_AILT_CS     = 0x04
_AIHT_CS     = 0x06
_PERS_CS     = 0x0C
_CONFIG_CS   = 0x0D
_CONTROL_CS  = 0x0F
_ID_CS       = 0x12
_STATUS_CS   = 0x13
_CDATA_CS    = 0x14
_RDATA_CS    = 0x16
_GDATA_CS    = 0x18
_BDATA_CS    = 0x1A

_AIEN_CS       = 0x10
_WEN_CS        = 0x08
_AEN_CS        = 0x02
_PON_CS        = 0x01

_GAINS_CS  = (1, 4, 16, 60)

PIN_SCL = 22
PIN_SDA = 21
PIN_POT = 36
PIN_NEXT_BTN = 2
PIN_PREV_BTN = 5
PIN_HIT_BTN = 0
PIN_IN1 = 12
PIN_IN2 = 14
PIN_ENA = 27
PIN_SERVO = 25
PIN_TRIG_LEFT = 33
PIN_ECHO_LEFT = 32
PIN_TRIG_RIGHT = 19
PIN_ECHO_RIGHT = 18

def map(x, in_min, in_max, out_min, out_max):
    percentage = ((x - in_min) * 100) / (in_max - in_min)
    value = (percentage * (out_max - out_min) / 100) + out_min
    return value

class Distance:
    def __init__(self):
        self.left_sensor = HCSR04(trigger_pin=PIN_TRIG_LEFT, echo_pin=PIN_ECHO_LEFT)
        self.right_sensor = HCSR04(trigger_pin=PIN_TRIG_RIGHT, echo_pin=PIN_ECHO_RIGHT)

    def right(self):
        return self.right_sensor.distance_cm()

    def left(self):
        return self.left_sensor.distance_cm()
        
class Display:
    def __init__(self):
        i2c = I2C(1, scl=Pin(PIN_SCL), sda=Pin(PIN_SDA))
        oled_width = 128
        oled_height = 64
        self.oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
        
    def show_text_list(self, text_list, selected=-1):
        
        self.oled.fill(0)
        for i, t in enumerate(text_list):
            if selected == i:
                self.oled.text('> ' + t, 0, i * 10)
            else:
                self.oled.text('  ' + t, 0, i * 10)
        self.oled.show()
        
    def show_text(self, text):
        self.oled.fill(0)
        self.oled.text(text, 0, 0)
        self.oled.show()

    def show_big_text(self, big_text):
        line = ""
        lines = []
        for c in big_text:
            if len(line) < 16:
                line += c
            else:
                lines.append(line)
                line = ""
                line += c
        if line != "":
            lines.append(line)
        print(lines)
        self.show_text_list(lines)
        
class Buttons:
    def __init__(self):
        self.next_button = Pin(PIN_NEXT_BTN, Pin.IN, Pin.PULL_UP) # DOWN
        self.hit_button = Pin(PIN_HIT_BTN, Pin.IN, Pin.PULL_UP) # CENTER
        self.previous_button = Pin(PIN_PREV_BTN, Pin.IN, Pin.PULL_UP) # UP
        
    def next_is_pressed(self):
        return self.next_button.value() == 0
    
    def hit_is_pressed(self):
        return self.hit_button.value() == 0
    
    def previous_is_pressed(self):
        return self.previous_button.value() == 0

    def any_is_pressed(self):
        return self.next_button.value() == 0 or self.hit_button.value() == 0 or self.previous_button.value() == 0
    
    def wait_next(self):
        while self.next_button.value() == 1:
            pass
            
    def wait_hit(self):
        while self.hit_button.value() == 1:
            pass
        
    def wait_previous(self):
        while self.previous_button.value() == 1:
            pass

    def wait_button(self, buttons=["HIT", "NEXT", "PREVIOUS"]):
        while True:
            if self.hit_is_pressed() and "HIT" in buttons:
                return "HIT"
            if self.next_is_pressed() and "NEXT" in buttons:
                return "NEXT"
            if self.previous_is_pressed() and "PREVIOUS" in buttons:
                return "PREVIOUS"
        
class Compass:
    __gain__ = {
        '0.88': (0 << 5, 0.73),
        '1.3':  (1 << 5, 0.92),
        '1.9':  (2 << 5, 1.22),
        '2.5':  (3 << 5, 1.52),
        '4.0':  (4 << 5, 2.27),
        '4.7':  (5 << 5, 2.56),
        '5.6':  (6 << 5, 3.03),
        '8.1':  (7 << 5, 4.35)
    }

    # Correction to be set after calibration
    xs=1
    ys=1
    xb=0
    yb=0

    def __init__(self, scl=22, sda=21, address=0x1e, gauss='1.9', declination=(4, 54), 
                    cnf_file="cnf.txt"):
        self.orientation = {'forward':0, "backward":0, "left":0, "right":0}
        self.correction_values = {"xs":1, "ys":1, "xb":0, "yb":0}
        self.i2c = i2c = I2C(1, scl=Pin(22), sda=Pin(21))
        self.address = address
        
        i2c.writeto_mem(self.address, 0x00, pack('B', 0b111000))

        reg_value, self.gain = self.__gain__[gauss]
        i2c.writeto_mem(self.address, 0x01, pack('B', reg_value))

        i2c.writeto_mem(self.address, 0x02, pack('B', 0x00))
        self.declination = (declination[0] + declination[1] / 60) * pi / 180

        self.data = array('B', [0] * 6)
        self.cnf_file = cnf_file

        self.read_cnf_file()
        self.current_orientation = "forward"

    def init_orientation(self):
        degrees = self.read_degrees()
        for ort in self.orientation:
            min_value = (359 + self.orientation[ort] - 10) % 360
            max_value = (self.orientation[ort] + 10) % 360
            if min_value > max_value:
                min_value, max_value = max_value, min_value
            if min_value < degrees < max_value:
                if ort == "right":
                    self.orientation["forward"], self.orientation["left"], self.orientation["back"], self.orientation["right"] = self.orientation["right"], self.orientation["back"], self.orientation["left"], self.orientation["forward"] 
                elif ort == "back":
                    self.orientation["forward"], self.orientation["left"], self.orientation["back"], self.orientation["right"] = self.orientation["back"], self.orientation["left"], self.orientation["forward"], self.orientation["right"] 
                elif ort == "left":
                    self.orientation["forward"], self.orientation["left"], self.orientation["back"], self.orientation["right"] = self.orientation["left"], self.orientation["forward"], self.orientation["right"], self.orientation["back"] 
            break

    def read(self):
        data = self.data
        gain = self.gain

        self.i2c.readfrom_mem_into(self.address, 0x03, data)
        
        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]

        x = x - (1 << 16) if x & (1 << 15) else x
        y = y - (1 << 16) if y & (1 << 15) else y
        z = z - (1 << 16) if z & (1 << 15) else z

        x = x * gain
        y = y * gain
        z = z * gain
        
        # Apply calibration corrections
        x = x * self.xs + self.xb
        y = y * self.ys + self.yb

        return x, y, z

    def heading(self, x, y):
        heading_rad = atan2(y, x)
        heading_rad += self.declination

        # Correct reverse heading.
        if heading_rad < 0:
            heading_rad += 2 * pi

        # Compensate for wrapping.
        elif heading_rad > 2 * pi:
            heading_rad -= 2 * pi

        # Convert from radians to degrees.
        heading = heading_rad * 180 / pi
        degrees = floor(heading)
        minutes = round((heading - degrees) * 60)
        return degrees, minutes

    def read_degrees(self):
        x, y, z = self.read()
        return self.heading(x, y)[0]

    def format_result(self, x, y, z):
        degrees, minutes = self.heading(x, y)
        return 'X: {:.4f}, Y: {:.4f}, Z: {:.4f}, Heading: {}° {} '.format(x, y, z, degrees, minutes)

    def read_cnf_file(self):
        self.orientation = {'forward':0, "backward":0, "left":0, "right":0}
        self.correction_values = {"xs":1, "ys":1, "xb":0, "yb":0}
        with open(self.cnf_file, 'a') as f:
            for line in f:
                #print(line)
                dict_line = loads(line)
                if dict_line["data"] == "compass":
                    self.orientation['forward'] = dict_line["forward"]
                    self.orientation['backward'] = dict_line["backward"]
                    self.orientation['left'] = dict_line["left"]
                    self.orientation['right'] = dict_line["right"]
                    self.correction_values['xs'] = dict_line["xs"]
                    self.correction_values['ys'] = dict_line["ys"]
                    self.correction_values['xb'] = dict_line["xb"]
                    self.correction_values['yb'] = dict_line["yb"]
                    break
                
    def write_cnf_file(self, orientation=None, corrections=None):
        replacement = ""
        # read file to change the values
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] == "compass":
                    if orientation is not None:
                        dict_line["forward"] = orientation["forward"]
                        dict_line["backward"] = orientation["backward"]
                        dict_line["left"] = orientation["left"]
                        dict_line["right"] = orientation["right"]
                    if corrections is not None:
                        dict_line["xs"] = corrections['xs']
                        dict_line["ys"] = corrections['ys']
                        dict_line["xb"] = corrections['xb']
                        dict_line["yb"] = corrections['yb']
                replacement = replacement + str(dict_line) + "\n"
        sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

class ColorSensor:
    """Driver for Grove I2C Color Sensor (TCS34725)"""

    def __init__(self, address=0x29, scl=22, sda=21, cnf_file="cnf.txt"):
        self.hsv_lines = None
        self.cnf_file = cnf_file
        self.i2c = I2C(1, scl=Pin(scl), sda=Pin(sda))
        
        self.address = address

        self.awake = False

        self.set_integration_time(24)
        self.set_gain(4)

        # read configuration file to update color ranges of lines
        self.read_cnf_file()

    def detect_color(self):
        # self.hsv_lines = {'white_min':0, "white_max":0, 'orange_min':0, "orange_max":0, "blue_min":0, "blue_max":0}
        hsv_value = self.hsv()[0]
        if self.hsv_lines["white_min"] <= hsv_value <= self.hsv_lines["white_max"]:
            return "white"
        if self.hsv_lines["orange_min"] <= hsv_value <= self.hsv_lines["orange_max"]:
            return "orange"
        if self.hsv_lines["blue_min"] <= hsv_value <= self.hsv_lines["blue_max"]:
            return "blue"
        return "unknown"
        
    def line_calibration_is_valid(self, new_hsv_lines):
        if new_hsv_lines == None:
            return False
        for i in range(0, 360):
            if i in range(new_hsv_lines['white_min'], new_hsv_lines['white_max'] + 1) and \
                i in range(new_hsv_lines['orange_min'], new_hsv_lines['orange_max'] + 1) and \
                    i in range(new_hsv_lines['blue_min'], new_hsv_lines['blue_max'] + 1):
                    return False
        return True

    def wakeup(self):
        enable = self._read_byte(_ENABLE_CS)
        self._write_byte(_ENABLE_CS, enable | _PON_CS | _AEN_CS)
        sleep(0.0024)

        self.awake = True

    def sleep(self):
        enable = self._read_byte(_ENABLE_CS)
        self._write_byte(_ENABLE_CS, enable & ~_PON_CS)

        self.awake = False

    def is_awake(self):
        return self._read_byte(_ENABLE_CS) & _PON_CS

    def set_wait_time(self, t):
        pass

    def id(self):
        return self._read_byte(_ID_CS)

    def integration_time(self):
        steps = 256 - self._read_byte(_ATIME_CS)
        return steps * 2.4

    def set_integration_time(self, t):
        """
        Set the integration time of the sensor
        """

        if t < 2.4:
            t = 2.4
        elif t > 614.4:
            t = 614.4
        
        steps = int(t / 2.4)
        self._integration_time = steps * 2.4
        self._write_byte(_ATIME_CS, 256 - steps)

    def gain(self):
        """
        The gain control. Should be 1, 4, 16, or 60.
        """

        return _GAINS_CS[self._read_byte(_CONTROL_CS)]

    def set_gain(self, gain):
        if gain in _GAINS_CS:
            self._write_byte(_CONTROL_CS, _GAINS_CS.index(gain))

    def raw(self):
        """
        Read RGBC registers
        return 16 bits red, green, blue and clear data
        """

        if not self.awake:
            self.wakeup()

        while not self._valid():
            sleep(0.0024)

        data = tuple(self._read_word(reg) for reg in (_RDATA_CS, _GDATA_CS, _BDATA_CS, _CDATA_CS))
        return data

    def hsv(self):
        r, g, b, clear = self.raw()
        
        if clear:
            r = r / clear
            g = g / clear
            b = b / clear
        else:
            r, g, b = 0, 0, 0

        #r, g, b = r / 255, g / 255, b / 255
        maxc = max(r, g, b)
        minc = min(r, g, b)
        v = maxc
        if minc == maxc:
            return 0.0, 0.0, v
        s = (maxc-minc) / maxc
        rc = (maxc-r) / (maxc-minc)
        gc = (maxc-g) / (maxc-minc)
        bc = (maxc-b) / (maxc-minc)
        if r == maxc:
            h = bc-gc
        elif g == maxc:
            h = 2.0+rc-bc
        else:
            h = 4.0+gc-rc
        h = (h/6.0) % 1.0
        return int(h*359), int(s*255), int(v*255)
        
    def rgb(self):
        """
        Read the RGB color detected by the sensor.  Returns a 3-tuple of
        red, green, blue component values as bytes (0-255).
        """

        r, g, b, clear = self.raw()
        
        if clear:
            r = int(255 * r / clear)
            g = int(255 * g / clear)
            b = int(255 * b / clear)
        else:
            r, g, b = 0, 0, 0
        return r, g, b

    def _valid(self):
        """
        Check if RGBC is valid
        """
        return self._read_byte(_STATUS_CS) & 0x01

    def _read_byte(self, address):
    
        command = _CMD_CS | address
        data = array('B', [0])
        self.i2c.readfrom_mem_into(self.address, command, data)
        return data[0]

    def _read_word(self, address):
        command = _CMD_CS | _AUTO_CS | address
        data = array('B', [0] * 2)
        self.i2c.readfrom_mem_into(self.address, command, data)#self.bus.read_word_data(self.address, command)
        
        return (data[0] << 8) | data[1]

    def _write_byte(self, address, data):
        command = _CMD_CS | address
        self.i2c.writeto_mem(self.address, command, pack('B', data)) # pack('B', 0b111000)
        #self.bus.write_byte_data(self.address, command, data)

    def _write_word(self, address, data):
        command = _CMD_CS | _AUTO_CS | address
        data = [(data >> 8) & 0xFF, data & 0xFF]
        self.i2c.writeto_mem(self.address, command, pack('B', data))
        #self.bus.write_i2c_block_data(self.address, command, data)

    def read_cnf_file(self):
        self.hsv_lines = {'white_min':0, "white_max":0, 'orange_min':0, "orange_max":0, "blue_min":0, "blue_max":0}
        with open(self.cnf_file, 'a') as f:
            for line in f:
                #print(line)
                dict_line = loads(line)
                if dict_line["data"] == "hsv_lines":
                    self.hsv_lines['white_min'] = dict_line["white_min"]
                    self.hsv_lines['white_max'] = dict_line["white_max"]
                    self.hsv_lines['orange_min'] = dict_line["orange_min"]
                    self.hsv_lines['orange_max'] = dict_line["orange_max"]
                    self.hsv_lines['blue_min'] = dict_line["blue_min"]
                    self.hsv_lines['blue_max'] = dict_line["blue_max"]
                    break
                
    def write_cnf_file(self, hsv_lines):
        replacement = ""
        # read file to change the values
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] == "hsv_lines":
                    dict_line["white_min"] = hsv_lines["white_min"]
                    dict_line["white_max"] = hsv_lines["white_max"]
                    dict_line["orange_min"] = hsv_lines["orange_min"]
                    dict_line["orange_max"] = hsv_lines["orange_max"]
                    dict_line["blue_min"] = hsv_lines["blue_min"]
                    dict_line["blue_max"] = hsv_lines["blue_max"]
                replacement = replacement + str(dict_line) + "\n"
        sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

class DC:
    def __init__(self):
        self.in1 = Pin(PIN_IN1, Pin.OUT)
        self.in1.value(0)
        self.in2 = Pin(PIN_IN2, Pin.OUT)
        self.in2.value(0)
        self.ena = PWM(Pin(PIN_ENA), freq=50, duty=0)
        self.pot = ADC(Pin(PIN_POT))
        self.pot.atten(ADC.ATTN_11DB)
        self.pot.width(ADC.WIDTH_10BIT)
        self.max_pwm = 1023
        self.forward = True

    def set_direction(self, forward=True):
        if forward:
            self.forward = True
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.forward = False
            self.in1.value(1)
            self.in2.value(0)

    def stop(self):
        self.in1.value(0)
        self.in2.value(0)

    def update_max_speed(self):
        self.max_pwm = self.pot.read()

    def read_pot(self):
        return self.pot.read()

    def move(self, speed=50, forward=True):
        speed_duty = int((self.max_pwm * speed) / 100)
        self.ena.duty(speed_duty)

class Servo:
    def __init__(self):
        self.cnf_file = "cnf.txt"
        self.left_duty = 0
        self.right_duty = 0
        self.forward_duty = 0 
        self.read_cnf_file()
        self.servo = PWM(Pin(PIN_SERVO), freq=50, duty=self.forward_duty)

    def steering(self, steer=0):
        duty = 0
        if steer == 0:
            duty = self.forward_duty
        else:
            duty = map(steer, -100, 100, self.left_duty, self.right_duty)
        self.servo.duty(duty)

    def set_duty(self, duty):
        self.servo.duty(duty)

    def read_cnf_file(self):
        with open(self.cnf_file, 'a') as f:
            for line in f:
                #print(line)
                dict_line = loads(line)
                if dict_line["data"] == "servo":
                    self.left_duty = dict_line["left"]
                    self.right_duty = dict_line["right"]
                    self.forward_duty = dict_line["forward"]
                    break
                
    def write_cnf_file(self, left, right, forward):
        replacement = ""
        # read file to change the values
        with open(self.cnf_file, 'r') as f:
            # using the for loop
            for line in f:
                dict_line = loads(line)
                if dict_line["data"] == "servo":
                    dict_line["left"] = left
                    dict_line["right"] = right
                    dict_line["forward"] = forward
                replacement = replacement + str(dict_line) + "\n"
        sleep(0.1)
        # re-write the file with the new values
        with open(self.cnf_file, 'w') as f:
            f.write(replacement.replace("\'", "\""))

class Car:
    def __init__(self, display, buttons, compass, color_sensor, dc, servo, distance):
        self.display = display
        self.buttons = buttons
        self.compass = compass
        self.color_sensor = color_sensor
        self.dc = dc
        self.servo = servo
        self.distance = distance
        self.max_speed_percent = map(self.dc.pot.read(), 0, 1023, 0, 100)

    


    def menu(self):
        main_options = cnf_options = {}
        # # Run 1 options
        # run1_options = {"GO":1, "BACK":"main_options"}
        # # Run 2 Options
        # run2_options = {"GO":2, "BACK":"main_options"}
        # Configuration options - compass
        compass_options = {"CNF Orientation":3, "MAG Calibration":4, "BACK":"cnf_options"}
        # Configuration options - color
        color_options = {"Calibrate Lines":5, "BACK":"cnf_options"}
        # Configuration options - pot
        pot_options = {"Calibrate Pot":9, "BACK":"cnf_options"}
        # Configuration options - servo
        servo_options = {"Calibrate SERVO":10, "BACK":"cnf_options"}
        # Configuration options
        cnf_options = {"Compass": compass_options, "Color":color_options, "Pot":pot_options, "Servo":servo_options, "BACK":"main_options"}
        # Info options
        info_options = {"Read Pot":6, "Read Color":7, "Read Compass":8, "Read HSV":11, "Read Distance":12,  "BACK":"main_options"}
        # Main menu options
        main_options = {"Configuration":cnf_options, "INFO":info_options}
        
        current_options = main_options
        exit_menu = False
        request = 0
        while not exit_menu:
            selected = 0
            option_list = list(current_options.keys())
            
            while True:
                self.display.show_text_list(option_list, selected=selected)
                if self.buttons.next_is_pressed():
                    selected += 1
                    if selected >= len(option_list):
                        selected = len(option_list) - 1
                    sleep(0.5)
                if self.buttons.previous_is_pressed():
                    selected -= 1
                    if selected < 0:
                        selected = 0
                    sleep(0.5)
                if self.buttons.hit_is_pressed():
                    option = current_options[option_list[selected]]
                    #print(option)
                    sleep(0.5)
                    if type(option) is dict:
                        # Next level of options
                        current_options = option
                        break
                    else:
                        if option_list[selected] == "BACK":
                            if current_options["BACK"] == "main_options":
                                current_options = main_options
                            elif current_options["BACK"] == "cnf_options":
                                current_options = cnf_options
                        request = option
                        #print("option:" + str(option))
                        exit_menu = True
                        break
                    
                # To allow time for OLED refresh
                sleep(0.1)
        self.main(request)

    def main(self, request=0):
        '''
        Via the main method runs all the car's processes
        Through the menu method in the end of the process, it is called recursively
        '''
        # if request == 1:
        #     self.run_1()
        # elif request == 2:
        #     self.run_2()
        if request == 3:
            self.calibrate_orientation()
        elif request == 4:
            self.calibrate_compass()
        elif request == 5:
            self.calibrate_colors()
        elif request == 6:
            self.show_pot()
        elif request == 7:
            self.show_color()
        elif request == 8:
            self.show_compass()
        elif request == 9:
            self.calibrate_speed()
        elif request == 10:
            self.calibrate_servo()
        elif request == 11:
            self.show_hsv()
        elif request == 12:
            self.show_distances()
            
        
        request=0
        self.menu()
        
    def show_pot(self):
        '''
        Shows the value of the potensiometer on the screen
        '''

        while not self.buttons.previous_is_pressed():
            txt_list = []
            txt_list.append("POT")
            txt_list.append("")
            txt_list.append(str(self.dc.read_pot()))
            self.display.show_text_list(txt_list)
            sleep(0.5)
        sleep(0.5)
        self.menu()

    def show_distances(self):
        '''
        Shows the value of the selected ultrasonic sensor on the screen
        '''
        
        while not self.buttons.previous_is_pressed():
            txt_list = []
            txt_list.append("Left:" + str(self.distance.left()))
            txt_list.append("")
            txt_list.append("Right:" + str(self.distance.right()))
            self.display.show_text_list(txt_list)
            sleep(0.5)
        sleep(0.5)
        self.menu()

    def show_compass(self):
        '''
        Shows the value of the compass sensor on the screen
        '''
        
        while not self.buttons.previous_is_pressed():
            txt_list = []
            txt_list.append("COMPASS")
            txt_list.append("")
            txt_list.append(str(self.compass.read_degrees()))
            txt_list.append("")
            txt_list.append("EXIT: UP")
            
            self.display.show_text_list(txt_list)
            sleep(0.5)
        sleep(0.5)
        self.menu()

    def show_hsv(self):
        '''
        Shows the value of the color sensor on the screen
        '''
        
        while not self.buttons.previous_is_pressed():
            txt_list = []
            txt_list.append("Color")
            h, s, v = self.color_sensor.hsv()
            txt_list.append("H:" + str(h))
            txt_list.append("S:" + str(s))
            txt_list.append("V:" + str(v))
            txt_list.append("")
            txt_list.append("EXIT: PREVIOUS")
            self.display.show_text_list(txt_list)
            sleep(0.5)
        sleep(0.5)
        self.menu()

    def show_color(self):
        while not self.buttons.previous_is_pressed():
            txt_list = []
            txt_list.append("Color")
            color = self.color_sensor.detect_color()
            txt_list.append(color)
            txt_list.append("")
            txt_list.append("EXIT: PREVIOUS")
            self.display.show_text_list(txt_list)
            sleep(0.1)
        sleep(0.5)
        self.menu()

    def calibrate_servo(self):
        self.servo.read_cnf_file()
        left_duty = right_duty = forward_duty = 0
        sleep(1)
        directions = ["left", "right", "forward"]
        for dir in directions:
            print("Current forward duty: ", str(self.servo.forward_duty))
            d = input("Test duty for " + dir + " (type 'next' for next position):")#map(self.dc.pot.read(), 0, 1023, 140, 40)
            #pr_value = d
            while d != "next":  
                d = int(d)      
                pr_value = d        
                txt_list = []
                txt_list.append("Steer " + dir)
                txt_list.append(str(d))
                self.display.show_text_list(txt_list)     
                self.servo.set_duty(d)
                d = input("Test duty for " + dir + " (type 'next' for next position):")#map(self.dc.pot.read(), 0, 1023, 140, 40)
                sleep(0.2)
            
            if dir == "left":
                left_duty = pr_value
            elif dir == "right":
                right_duty = pr_value
            elif dir == "forward":
                forward_duty = pr_value
            sleep(1)
        sleep(1)
        txt = []
        txt.append("SERVO VALUES")
        txt.append("LEFT:" + str(left_duty))
        txt.append("FRD:" + str(forward_duty))
        txt.append("RIGHT:" + str(right_duty))
        txt.append("HIT: SAVE")
        txt.append("DOWN: CANCEL")
        self.display.show_text_list(txt)
        btn = self.buttons.wait_button(["NEXT", "HIT"])
        if btn == "HIT":
            self.servo.write_cnf_file(left=left_duty, right=right_duty, forward=forward_duty)
            self.display.show_big_text("Servo calibration and cnf file update completed succesfully! Wait 5 sec") 
            sleep(0.5)
            self.servo.read_cnf_file()
        else:
            self.display.show_big_text("Servo calibration CANCELED! Nothing changed in cnf file ... Wait 5 sec") 
        sleep(5)
        self.display.show_big_text("DO NOT forget to adjust max speed with pot!!! ... Wait 5 sec") 

    def calibrate_speed(self):
        self.max_speed_percent = map(self.dc.pot.read(), 0, 1023, 0, 100)
        while not self.buttons.hit_is_pressed():
            txt_list = []
            txt_list.append("Speed % " + str(self.max_speed_percent))
            txt_list.append("CENTER to exit")
            self.display.show_text_list(txt_list)     
            self.max_speed_percent = map(self.dc.pot.read(), 0, 1023, 0, 100)
            sleep(0.2)
        self.display.show_big_text("Max speed: " + str(self.max_speed_percent) + "! Wait 5 sec") 
        sleep(5)
    
    def calibrate_compass(self):
        Xmin=1000
        Xmax=-1000
        Ymin=1000
        Ymax=-1000
        self.display.show_big_text("Rotate carefully the magnetometer and press CENTER to stop")
        while not self.buttons.hit_is_pressed():
            sleep(0.2)
            x, y, z = self.compass.read()
            Xmin=min(x,Xmin)
            Xmax=max(x,Xmax)
            Ymin=min(y,Ymin)
            Ymax=max(y,Ymax)
            txt_list = []
            txt_list.append("Xmin:" + str(Xmin))
            txt_list.append("Xmax:" + str(Xmax))
            txt_list.append("Ymin:" + str(Ymin))
            txt_list.append("Ymax:" + str(Ymax))
            txt_list.append("EXIT: HIT")
            self.display.show_text_list(txt_list)
        xs=1
        ys=(Xmax-Xmin)/(Ymax-Ymin)
        xb =xs*(1/2*(Xmax-Xmin)-Xmax)
        yb =xs*(1/2*(Ymax-Ymin)-Ymax)
        txt_list = []
        txt_list.append("Compass Results:")
        txt_list.append("xs:" + str(xs))
        txt_list.append("xb:" + str(xb))
        txt_list.append("ys:" + str(ys))
        txt_list.append("yb:" + str(yb))
        txt_list.append("C:SAVE-D:CANCEL")
        self.display.show_text_list(txt_list)
        btn = self.buttons.wait_button(["HIT", "NEXT"])

        if btn == "NEXT":
            txt = []
            txt.append("COMPASS CAL/TION")
            txt.append("CANCELED")
            txt.append("CENTER:stop process")
            self.display.show_text_list(txt)
            sleep(2)
            self.buttons.wait_hit()
        elif btn == "HIT":
            txt = []
            txt_list.append("xs:" + str(xs))
            txt_list.append("xb:" + str(xb))
            txt_list.append("ys:" + str(ys))
            txt_list.append("yb:" + str(yb))
            txt.append("CENTER:edit CNF")
            txt.append("DOWN:cancel")
            self.display.show_text_list(txt)
            btn = self.buttons.wait_button(["NEXT", "HIT"])
            sleep(1)
            
            if btn == "HIT":
                self.compass.write_cnf_file(corrections={"xs":xs, "ys":ys, "xb":xb, "yb":yb})
                self.display.show_big_text("Compass calibration and cnf file update completed succesfully! Wait 5 sec") 
                sleep(0.5)
                self.compass.read_cnf_file()
            else:
                self.display.show_big_text("Compass calibration CANCELED! Nothing changed in cnf file ... Wait 5 sec") 
            sleep(5)
        
    def calibrate_colors(self):
        new_hsv_lines = {'white_min':0, 'white_max':0, 'orange_min':0, "orange_max":0, "blue_min":0, "blue_max":0}
        colors = ["WHITE", "ORANGE", "BLUE"]
        
        for color in colors:
            values = []
            while not self.buttons.previous_is_pressed():
                value = self.color_sensor.hsv()[0]
                txt = []
                txt.append(color)
                txt.append("GET VALUE AND")
                txt.append("PRESS CENTER")                
                txt.append(str(value))
                txt.append("CENTER: SAVE")
                txt.append("UP: NEXT COLOR")
                self.display.show_text_list(txt)
                if self.buttons.hit_is_pressed():
                    values.append(value)
                    txt = []                    
                    txt.append("VALUE SAVED")
                    txt.append("FOR " + color)                
                    txt.append(str(value))
                    txt.append("WAIT 3 sec")
                    self.display.show_text_list(txt)
                    sleep(3)
            if color == "ORANGE":
                new_hsv_lines["orange_min"] = min(values)
                new_hsv_lines["orange_max"] = max(values)
            elif color == "BLUE":
                new_hsv_lines["blue_min"] = min(values)
                new_hsv_lines["blue_max"] = max(values)
            elif color == "WHITE":
                new_hsv_lines["white_min"] = min(values)
                new_hsv_lines["white_max"] = max(values)
            sleep(1)
        if self.color_sensor.line_calibration_is_valid(new_hsv_lines):
            txt = []                    
            txt.append("VALID")
            txt.append("W:" + str(new_hsv_lines["white_min"]) + "_" + str(new_hsv_lines["white_max"]))
            txt.append("O:" + str(new_hsv_lines["orange_min"]) + "_" + str(new_hsv_lines["orange_max"]))
            txt.append("B:" + str(new_hsv_lines["blue_min"]) + "_" + str(new_hsv_lines["blue_max"]))
            txt.append("CENTER: APPROVE")
            txt.append("DOWN: CANCEL")
            self.display.show_text_list(txt)
            btn = self.buttons.wait_button(["HIT", "NEXT"])
            if btn == "HIT":
                self.color_sensor.write_cnf_file(new_hsv_lines)
                txt = []                    
                txt.append("COLORS UPDATED!")
                txt.append("PRESS CENTER")
                self.display.show_text_list(txt)
                sleep(0.5)
                self.buttons.wait_hit()
            else:
                txt = []                    
                txt.append("CAL/TION CANCELED")
                txt.append("PRESS CENTER")
                self.display.show_text_list(txt)
                sleep(0.5)
                self.buttons.wait_hit()
        else:
            txt = []                    
            txt.append("!!!NOT VALID!!!")
            txt.append("W:" + str(new_hsv_lines["white_min"]) + "_" + str(new_hsv_lines["white_max"]))
            txt.append("O:" + str(new_hsv_lines["orange_min"]) + "_" + str(new_hsv_lines["orange_max"]))
            txt.append("B:" + str(new_hsv_lines["blue_min"]) + "_" + str(new_hsv_lines["blue_max"]))
            txt.append("PRESS CENTER")
            self.display.show_text_list(txt)
            sleep(0.5)
            self.buttons.wait_hit()
                    
    def calibrate_orientation(self):
        # Initialise dictionary
        orientation = {"forward":0, "backward":0, "left":0, "right":0}
        cancel = False
        for orin in ['forward', "backward", "left", "right"]:
            while not self.buttons.hit_is_pressed():
                txt = []
                txt.append("Compass Cal/tion")
                txt.append(orin)
                txt.append(str(self.compass.read_degrees()))
                txt.append("CENTER: CONTINUE")
                txt.append("UP: CANCEL")
                self.display.show_text_list(txt)                
                if self.buttons.previous_is_pressed():
                    cancel = True
                    break
                sleep(0.3)
            if cancel:
                break
            orientation[orin] = self.compass.read_degrees()
            txt = []
            txt.append("Compass Cal/tion")
            txt.append(orin + " value:")
            txt.append(str(orientation[orin]))
            txt.append("CENTER: NEXT")
            txt.append("UP: CANCEL")
            self.display.show_text_list(txt)
            btn = self.buttons.wait_button(["PREVIOUS", "HIT"])
            sleep(0.5)
        if btn == "PREVIOUS":
            txt = []
            txt.append("COMPASS CAL/TION")
            txt.append("CANCELED")
            txt.append("CENTER:stop process")
            self.display.show_text_list(txt)
            sleep(2)
            self.buttons.wait_hit()
        elif btn == "HIT":
            txt = []
            txt.append("F: " + str(orientation['forward']))
            txt.append("R: " + str(orientation['right']))
            txt.append("B: " + str(orientation['backward']))
            txt.append("L: " + str(orientation['left']))
            txt.append("CENTER:edit CNF")
            txt.append("UP:cancel")
            self.display.show_text_list(txt)
            btn = self.buttons.wait_button(["PREVIOUS", "HIT"])
            sleep(0.5)
            
            if btn == "HIT":
                self.compass.write_cnf_file(orientation=orientation)
                self.display.show_big_text("Compass calibration and cnf file update completed succesfully! Wait 5 sec") 
                sleep(0.5)
                self.compass.read_cnf_file()
            else:
                self.display.show_big_text("Compass calibration CANCELED! Nothing changed in cnf file ... Wait 5 sec") 
            sleep(5)

def main():
    car = Car(Display(), Buttons(), Compass(), ColorSensor(), DC(), Servo(), Distance())
    car.main()

    
if __name__ == "__main__":
    main()
    
    

