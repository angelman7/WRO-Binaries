# WRO - NameNotFound

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2022 for the team NameNotFound.

## Content
* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members).
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom).
* `video` contains the video.md file with the link to a video where driving demonstration exists.
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition.
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction
First things first, the hardware consinsts of a Jetson Nano (running Linux) thats is the "mind" of the robot as it is where all the code is run.

Concerning the movement of the vehicle, a DC and a servo motor are used. The DC motor is responsible for the speed of the car while the servo motor is responsible for the steering of the car.

The Jetson Nano also controls all the sensors used in our project:
* `1 color sensor` which detects the orange and blue lines
* `1 OLED screen` which displays some info about the vehicle
* `3 selection buttons` which help navigate through the menu and options of the OLED display and execute any program
* `1 camera` which is used to detect what is in front of the vehicle
     
We are also using a Jetson Nano using the Linux OS, which is using all the input sensors mentioned above

On the software part the following programs and modules are used:
   - `car.py` contains the main program that will run during the competition using the modules mentioned below.
   - `cnf.txt` contains many variables used by the car.py that were found through contious testing, like the color codes of the lines etc.
   - `modules`
       - `bmm150.py`
       - `button.py` contains the class used to control the buttons that is used to detect whether or not a button is pressed.
       - `camera.py` contains the class used to read the camera.
       - `color.py` contains the class used for the color sensor and to check what it detects.
       - `compass.py` contains the class used to controll the compass and read the current orientation.
       - `cvtool.py` contains the class used to manage the camera and detect what and what type of obstacle is in front of the vehicle.
       - `drive.py` contains the class used for control and management of both the DC and servo.
       - `grove_i2c_bus.py` contains the bus used by the program grove_i2c_color_sensor_v2.py.
       - `grove_i2c_color_sesor_v2.py` contains the class used to control the I2C communication for both the OLED display and the Color Sensor.
       - `line2turn.py` contains the class used to detect and recognise the blue and orange lines on the field ground.
       - `oled.py` contains the class used to control the OLED display and showw text on it.
       - `oled_sample.py` a program used to debug the OLED display.
       - `pillar.py` contains the class used by the camera to manage and detect and avoid any pillars that might be in front of the vehicle correctly.
       - `wall.py` contains the class used by the camera to manage and detect and avoid any walls that might be in front of the vehicle correctly imminent.
   
 **Car.py**
This is the main program that runs during both the run 1 and the run2 challenge. It starts by displaying a menu screen through the OLED display from which one can choose and execute, using the three buttons, any of the following options: (Each option is a separate function inside Car.py)
 
**Run 1:** (runs the program for the first challenge)

**Run 2:** (runs the program for the second challenge)
**Info:**
This function shows the current values of sensors while the program is running. This can be helpful for debugging reasons.
Such sensors might be:

**Configuration:**
This function is used to calibrate each sensor: 
 - The `color sensor` by taking various captures of each line on the field to create a range of hsv values for each color respectively.
 - The `compass` by spinning the vehicle around in various directions and axes for a few seconds so that the compass "gets a grasp" of its surroundings.
 - The `compass orientation` by matching values to all 4 sides of the field in order for the robot to know which side it starts from during both run 1 and run 2 challenges.
 - The `camera` by taking photos of the pillars in order to then manually create a range of hsv values so that the camera recognises the color of each pillar, and by taking photos of the walls to proceed through the same process to detect the walls around and inside the field.


**Cnf.txt**
[...]

**Modules**
[...]

# Credits - Libraries

## Grove I2C Bus
**License**

The MIT License (MIT)

Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
Copyright (C) 2018  Seeed Technology Co.,Ltd. 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

## Grove I2C Color Sensor
Library for Grove - I2C Color Sensor V2(https://www.seeedstudio.com/Grove-I2C-Color-Sensor-V2-p-2890.html)
