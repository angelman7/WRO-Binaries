from re import T
import threading
import RPi.GPIO as GPIO
from modules.button import Button
from modules.distance import Distance
from modules.drive import Drive
from modules.oled import OLED
from modules.cvtool import CVTool
from modules.line import Line
from modules.pillar import Pillar
from modules.wall import Wall
# from modules.line2turn import Line2Turn
from modules.color import Color
from modules.compass import Compass
from statistics import median
import time
import logging
import os

SERVO_GPIO = 32
IN1_GPIO = 35
IN2_GPIO = 37
ENA_GPIO = 33

TRIGGER_LEFT_GPIO = 22
ECHO_LEFT_GPIO = 24
TRIGGER_RIGHT_GPIO = 26
ECHO_RIGHT_GPIO = 23

BUTTON_UP_GPIO = 12
BUTTON_DOWN_GPIO = 18
BUTTON_CENTER_GPIO = 16

class Car:
    def __init__(self, level=logging.DEBUG, cnf_file="cnf.txt"):
        path = os.path.dirname(os.path.realpath(__file__))
        # base_path = os.path.abspath(os.path.join(path, os.pardir)) 
        self.cnf_file = os.path.join(path, cnf_file) 
        GPIO.cleanup()

        # Create objects
        self.drive = Drive(SERVO_GPIO, IN1_GPIO, IN2_GPIO, ENA_GPIO, self.cnf_file)
        self.left_distance = Distance(TRIGGER_LEFT_GPIO, ECHO_LEFT_GPIO)
        self.right_distance = Distance(TRIGGER_RIGHT_GPIO, ECHO_RIGHT_GPIO)
        self.button_up = Button(BUTTON_UP_GPIO)
        self.button_down = Button(BUTTON_DOWN_GPIO)
        self.button_center = Button(BUTTON_CENTER_GPIO) 
        self.oled = OLED()
        # self.line2turn = Line2Turn(cnf_file=self.cnf_file)
        self.cvtool = CVTool(use_cam=True, cnf_file=self.cnf_file)
        self.color = Color()
        self.compass = Compass(self.cnf_file)

        # # CVTool objects
        self.orange_line = Line(self.cvtool, "orange", filename=self.cnf_file)
        self.blue_line = Line(self.cvtool, "blue", filename=self.cnf_file)
        self.red_pillar = Pillar(self.cvtool, "red", filename=self.cnf_file)
        self.green_pillar = Pillar(self.cvtool, "green", filename=self.cnf_file)
        self.wall = Wall(self.cvtool, filename=self.cnf_file)

        # Useful properties
        self.run_direction = 0
        self.run_check_lines = False
        self.turn = False

        # Configure log file
        log_file_path = os.path.dirname(__file__) + "/logfile.log"
        logging.basicConfig(filename=log_file_path,
                            format="%(asctime)s %(message)s",
                            filemode='w')
        self.logger = logging.getLogger()
        self.logger.setLevel(level)
        self.logger.debug("Car object created")

    def menu(self):
        main_options = {}
        cnf_options = {}
        # Run 1 Options
        run1_options = {"BACK":"main_options", "GO":1}
        # Run 2 Options
        run2_options = {"BACK":"main_options", "GO":2}
        # Info options
        info_options = {"BACK":"main_options", "Read Distances":3, "Read Color":4, "Read Compass":5}
        # Configuration options
        cnf_options = {"BACK":"main_options", "Calibtate Color":6, "Calibrate Orientation":7, "Calibrate Compass":8, "Get Photos":9}
        # Main menu options
        main_options = {"RUN 1":run1_options, "RUN 2":run2_options, "INFO":info_options, "CONFIGURATION":cnf_options}
        
        current_options = main_options
        exit_menu = False
        request = 0
        while not exit_menu:
            selected = 0
            option_list = list(current_options.keys())
            update_display = True
            while True:
                if update_display:
                    self.oled.show_text_list(txt_list=option_list, selected=selected)
                    update_display = False
                 
                 
                if self.button_down.is_pressed():
                    print("down")
                    selected += 1
                    if selected >= len(option_list):
                        selected = len(option_list) - 1
                    update_display = True
                    time.sleep(0.5)
                if self.button_up.is_pressed():
                    print("up")
                    selected -= 1
                    if selected < 0:
                        selected = 0
                    update_display = True
                    time.sleep(0.5)
                if self.button_center.is_pressed():
                    option = current_options[option_list[selected]]
                    time.sleep(0.5)
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
                        exit_menu = True
                        break
                    
                # To allow time for OLED refresh
                # time.sleep(0.1)
        self.run_request(request)

    def run_request(self, request):
        print("Run request: ", request)

        if request == 1:
            self.logger.debug("New request: RUN 1")
            # self.run_1()
            # self.run_distance_sensors()
            self.run_1(speed=55)
        elif request == 2:
            self.logger.debug("New request: RUN 2")
            self.run_2()
        elif request == 3:
            self.logger.debug("New request: Read Distances")
            self.read_distances()
        elif request == 4:
            self.logger.debug("New request: Read Color")
            self.read_color()
        elif request == 5:
            self.logger.debug("New request: Read Compass")
            self.read_compass()
        elif request == 6:
            self.logger.debug("New request: Calibrate Color Sensor")
            self.calibrate_lines()
        elif request == 7:
            self.logger.debug("New request: Calibrate Orientations")
            self.calibrate_orientations()
        elif request == 8:
            self.logger.debug("New request: Calibrate Compass")
            self.callibrate_compass()
        elif request == 9:
            self.logger.debug("New request: Get Photos")
            self.get_photos()
    
        self.menu()

    def read_distances(self):
        txt = ["", ""]
        while not self.button_down.is_pressed():
            txt[0] = "L:" + str(self.left_distance.distance_cm(tries=3))
            txt[1] = "R:" + str(self.right_distance.distance_cm(tries=3))
            self.oled.show_text_list(txt)
            time.sleep(1)

    def run_distance_sensors(self, rounds=3, speed=60):
        # print("GO")
        kp = 3
        min_distance = 120
        self.compass.init_orientation()
        compass_target = self.compass.orientations[self.compass.current_target_index]
        self.drive.move(speed)
        turn = False
        turn_delay = 60000
        last_turn = 0
        # time.sleep(20)
        on_turn = False
        turn_number = 0
        round_number = 1
        self.oled.show_text_list(["ROUND " + str(round_number), "TURN " + str(turn_number)])
        while round_number < rounds:
            if self.button_down.is_pressed():
                self.drive.set_steering(0)
                self.drive.stop()
                self.oled.show_text("STOPPED!")
                return None
            current_ms = time.time() * 1000
            compass_error =  compass_target - self.compass.heading()
            if compass_error > 180:
                compass_error = compass_error - 360
            elif compass_error < -180:
                compass_error = 360 + compass_error
            steering = max(-100, min(100, kp*compass_error))
            # print(self.left_distance.distance_cm(tries=5))
            self.drive.smooth_steering(new_steer=steering, max_angle=20)
            if not on_turn:
                
                # print(left_distance, right_distance)
                
                if self.run_direction == 0:
                    left_distance = self.left_distance.distance_cm(tries=3) 
                    right_distance = self.right_distance.distance_cm(tries=3)
                    # self.oled.show_text_list(["L" + str(left_distance), "R" + str(right_distance)])
                    if left_distance > min_distance:
                        self.run_direction = -1
                        turn = True
                    elif right_distance > min_distance:
                        self.run_direction = 1
                        turn = True
                elif self.run_direction == -1:
                    left_distance = self.left_distance.distance_cm(tries=3) 
                    # self.oled.show_text_list(["L" + str(left_distance)])
                    if left_distance > min_distance:                    
                        turn = True
                elif self.run_direction == 1:
                    right_distance = self.right_distance.distance_cm(tries=3)
                    # self.oled.show_text_list(["R" + str(right_distance)])
                    if right_distance > min_distance:                    
                        turn = True
            if turn:
                print("turn", self.run_direction)
                self.compass.turn(direction=self.run_direction)
                compass_target = self.compass.orientations[self.compass.current_target_index]
                turn = False
                last_turn = current_ms
                on_turn = True
                self.oled.show_text("ON TURN " + str(self.run_direction))
                turn_number += 1
                if turn_number == 5:
                    turn_number = 0
                    round_number += 1
            elif on_turn and abs(compass_error) < 15:
                on_turn = False
                self.oled.show_text_list(["ROUND " + str(round_number), "TURN " + str(turn_number)])

    def check_lines(self):
        self.run_check_lines = True
        while self.run_check_lines:
            line = self.color.detect_line()
            if line != 0:
                if self.run_direction == line:
                    self.turn = True
                elif self.run_direction == 0:                    
                    self.run_direction = line
                    self.turn = True
                    
    def run_1(self, rounds=3, speed=80):
        kp = 3.5
        self.run_direction = 0
        print("a")
        self.compass.init_orientation()
        print("b")
        compass_target = self.compass.orientations[self.compass.current_target_index]
        t = threading.Thread(target=self.check_lines, args=())
        t.start()
        self.drive.move(speed)
        on_turn = False
        turn_number = 0
        round_number = 1
        
        while round_number <= rounds:
            if self.button_down.is_pressed() or self.button_center.is_pressed() or self.button_up.is_pressed():
                self.drive.set_steering(0)
                self.drive.stop()
                self.oled.show_text("STOPPED!")
                self.run_check_lines = False
                return None
            
            compass_error =  compass_target - self.compass.heading()
            if compass_error > 180:
                compass_error = compass_error - 360
            elif compass_error < -180:
                compass_error = 360 + compass_error
            steering = max(-100, min(100, kp*compass_error))
            self.drive.smooth_steering(new_steer=steering, max_angle=20)
            
            if self.turn and not on_turn:
                inner_wall = True
                if self.run_direction == -1 and self.left_distance.distance_cm(tries=3) > 90:
                    inner_wall = False
                elif self.run_direction == 1 and self.right_distance.distance_cm(tries=3) > 90:
                    inner_wall = False
                if not inner_wall:
                    self.compass.turn(direction=self.run_direction)
                    compass_target = self.compass.orientations[self.compass.current_target_index]
                    self.turn = False
                    on_turn = True
                    # self.oled.show_text("ON TURN " + str(self.run_direction))
                    turn_number += 1
                    if turn_number == 4:
                        turn_number = 0
                        round_number += 1
                    self.drive.set_steering(self.run_direction * 100)
            elif on_turn and abs(compass_error) < 15:
                on_turn = False
                self.turn = False
                # self.oled.show_text_list(["ROUND " + str(round_number), "TURN " + str(turn_number)])
        self.run_check_lines = False
        last_round_time = 5 #seconds
        start_last_round = time.time()
        while time.time() - start_last_round < last_round_time:
            compass_error =  compass_target - self.compass.heading()
            if compass_error > 180:
                compass_error = compass_error - 360
            elif compass_error < -180:
                compass_error = 360 + compass_error
            steering = max(-100, min(100, kp*compass_error))
            # print(self.left_distance.distance_cm(tries=5))
            self.drive.smooth_steering(new_steer=steering, max_angle=20) 
        self.drive.stop()


    def run_2(self):
        running = True
        while running:
            pass

    def read_color(self):
        previous_line = 0
        self.oled.show_text("Line: " + str(previous_line))
        while not self.button_down.is_pressed():
            h, s, v = self.color.read_hsv()
            line = 0
            for color_key in self.color.lines.keys():
                if self.color.lines[color_key]['hmin'] <= h <= self.color.lines[color_key]['hmax'] and \
                    self.color.lines[color_key]['smin'] <= s <= self.color.lines[color_key]['smax'] and \
                    self.color.lines[color_key]['vmin'] <= v <= self.color.lines[color_key]['vmax']:
                    if color_key == "orange":
                        line = 1
                    else:
                        line = -1
            
            self.oled.show_text_list([str((h, s, v)), str(line)])

            # line = self.color.detect_line()
            # if line != previous_line:
            #     self.oled.show_text("Line: " + str(line))
            #     previous_line = line
            time.sleep(2)

    def read_compass(self):
        if self.compass.orientations == None:
            self.oled.show_text_list(["CENTER: ORIENTATION"])
            time.sleep(1)
            while not self.button_center.is_pressed():
                pass
            self.compass.init_orientation()
        txt = [str(self.compass.orientations), "heading", "target"]
        txt[2] = str(self.compass.orientations[self.compass.current_target_index])
        previous_heading = self.compass.heading() 
        while not self.button_down.is_pressed():
            current_heading = self.compass.heading() 
            if current_heading != previous_heading:
                txt[1] = str(current_heading)
                previous_heading = current_heading
                self.oled.show_text_list(txt)
                # print(txt)

    def calibrate_servo(self):
        directions = ["LEFT", "CENTER", "RIGHT"]
        current_per = self.drive.forward_per
        self.drive.set_steering(current_per)
        txt_list = []
        steering = 0
        update_display = True
        txt_list = []
        txt_list.append("Left percent:" + str(self.drive.left_per))
        txt_list.append("Center percent:" + str(self.drive.forward_per))
        txt_list.append("Right percent:" + str(self.drive.right_per))
        self.oled.show_text_list(txt_list)
        time.sleep(4)
        for d in directions:
            print(d)
            stuck_on_value = True
            steering = 0 

            while stuck_on_value:
                save = False
                # print("CALIBRATE DIRECTION:", d)
                current_per = self.drive.forward_per
                self.drive.set_steering(current_per)
                txt_list = []
                txt_list.append("Calibrating Direction")
                txt_list.append(str(d) + ": " + str(steering))
                if update_display: 
                    self.oled.show_text_list(txt_list)
                    update_display = False
                time.sleep(2)

                if update_display:           
                    self.oled.show_text_list(txt_list)
                    update_display = False
                
                if self.button_center.is_pressed():
                    save = False
                    stuck_on_value = False
                    # txt_list = []
                    # txt_list.append("Moving on") 
                    self.oled.show_text_list(txt_list)
                    time.sleep(3)
                    update_display = True
                    break
                elif self.button_up.is_pressed():
                    steering+=.2
                elif self.button_down.is_pressed():
                    steering-=.2
                
                self.drive.set_steering(steering)
                
                if save:
                    txt_list = []
                    if d == "LEFT":
                        self.drive.left_per = steering
                        txt_list.append(f"'{d}':" + str(self.drive.left_per)) 
                    elif d == "CENTER":
                        self.drive.forward_per = steering
                        txt_list.append(f"'{d}':" + str(self.drive.forward_per)) 
                    elif d == "RIGHT":
                        self.drive.right_per = steering
                        txt_list.append(f"'{d}':" + str(self.drive.right_per)) 
                    self.oled.show_text_list(txt_list)
                    time.sleep(2)
                    update_display = True
                    self.drive.write_cnf_file(self.drive.left_per, self.drive.right_per, self.drive.forward_per, self.drive.speed)         

    def calibrate_lines(self):
        color_names = self.color.lines.keys()
        for color_name in color_names:
            hmin = 359
            smin = vmin = 255
            hmax = smax = vmax = 0
            while not self.button_down.is_pressed():
                hsv = self.color.read_hsv()
                txt_list = []
                txt_list.append("CENTER to keep value")
                txt_list.append("DOWN for next color")
                txt_list.append("MIN: " + str(hmin) + " " + str(smin) + " " +  str(vmin))
                txt_list.append("MIN: " + str(hmax) + " " + str(smax) + " " +  str(vmax))
                txt_list.append(str(hsv))
                txt_list.append("Calibrate " + color_name)
                self.oled.show_text_list(txt_list)
                if self.button_center.is_pressed():
                    if hsv[0] < hmin:
                        hmin = hsv[0]
                    if hsv[1] < smin:
                        smin = hsv[1]
                    if hsv[2] < vmin:
                        vmin = hsv[2]
                    if hsv[0] > hmax:
                        hmax = hsv[0]
                    if hsv[1] > smax:
                        smax = hsv[1]
                    if hsv[2] > vmax:
                        vmax = hsv[2]
            txt_list= []
            txt_list.append("CENTER to save to file")
            txt_list.append("UP to ignore values")
            txt_list.append("Color: " + color_name)
            txt_list.append("MIN: " + str(hmin) + " " + str(smin) + " " +  str(vmin))
            txt_list.append("MIN: " + str(hmax) + " " + str(smax) + " " +  str(vmax))
            self.oled.show_text_list(txt_list)
            while True:
                if self.button_up.is_pressed():
                    txt_list= []
                    txt_list.append(color_name + " calibration canceled")
                    txt_list.append("Wait 2\"")
                    self.oled.show_text_list(txt_list)
                    break
                if self.button_center.is_pressed():
                    self.color.lines[color_name]['hmin'] = hmin
                    self.color.lines[color_name]['hmax'] = hmax
                    self.color.lines[color_name]['smin'] = smin
                    self.color.lines[color_name]['smax'] = smax
                    self.color.lines[color_name]['vmin'] = vmin
                    self.color.lines[color_name]['vmax'] = vmax
                    self.color.write_cnf_file(color_name)
                    txt_list= []
                    txt_list.append(color_name + " calibration saved")
                    txt_list.append("Wait 2\"")
                    self.oled.show_text_list(txt_list)
                    break

    def calibrate_orientations(self):        
        for index in range(4):
            for j in range(2):
                txt_list = ["", "", "", "", ""]
                txt_list[0] = str(self.compass.orientations_1)
                txt_list[1] = str(self.compass.orientations_2)
                txt_list[2] = "Part " + str(index + 1) + "/4, side " + str(j + 1) + "/2" 
                txt_list[3] = "CENTER to save"
                txt_list[4] = "DOWN to cancel"
                self.oled.show_text_list(txt_list)
                while True:
                    heading = self.compass.heading()                    
                    if self.button_center.is_pressed():
                        if j == 0:
                            self.compass.orientations_1[index] = heading
                        else:
                            self.compass.orientations_2[index] = heading
                        break
                    if self.button_down.is_pressed():
                        return None   

        txt_list = []
        txt_list.append("Cal/ted values:")
        txt_list.append(str(self.compass.orientations_1))
        txt_list.append(str(self.compass.orientations_2))
        txt_list.append("Write: CENTER")
        txt_list.append("Cancel: DOWN")
        self.oled.show_text_list(txt_list)
        while True:
            if self.button_down.is_pressed():
                self.compass.read_cnf_file()
                txt_list = []
                txt_list.append("Calibration canceled")
                self.oled.show_text_list(txt_list)
                break
            if self.button_center.is_pressed():
                self.compass.write_cnf_file()
                txt_list = []
                txt_list.append("Calibration saved")
                self.oled.show_text_list(txt_list)
                break

    def callibrate_compass(self, seconds=60):
        msg = "Press center to start"
        self.oled.show_text(msg)
        self.button_center.wait()
        msg = ["Role the model from all", "over directions", "for " + str(seconds) + " seconds"]
        self.oled.show_text_list(txt_list=msg)
        self.compass.calibrate(seconds)
        msg = ["CENTER: save values", "DOWN: cancel calibration"]        
        self.oled.show_text_list(msg)
        while True:
            if self.button_center.is_pressed():
                self.compass.write_cnf_file()
                msg = ["Compass calibration saved", "Wait 3 seconds ..."]
                time.sleep(3)        
                self.oled.clear()
                break
            if self.button_down.is_pressed():
                msg = ["Compass calibration CANCELED!", "Wait 3 seconds ..."]
                time.sleep(3)        
                self.oled.clear()
                break

    def follow_compass(self, kp=1, max_steering=35):
        compass_error = self.compass_tool.heading() - self.compass_tool.get_current_target()
        if compass_error > 180:
            compass_error = compass_error - 360
        elif compass_error < -180:
            compass_error = 360 + compass_error
        steering = max(-max_steering, min(max_steering, -compass_error * kp))
        # print("compass_error:", compass_error, "steering", steering)

        # self.steer(steering)
        self.smooth_steering(new_steer=steering, max_angle=10)

    def get_photos(self):
        pass   


def main():
    car = Car()
    car.menu()
    # car.run_tassos()
    # car.callibrate_compass()
    # time.sleep(2)


if __name__== "__main__":
    main()
