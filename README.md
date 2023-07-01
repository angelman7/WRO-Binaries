# WRO - Binaries

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2022 for the team Binaries.

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

The Jetson Nano also utilises the power of R.O.S.(Robot Operating System) which is a set of software libraries and tools that let us build robot applications as this. From drivers to state-of-the-art algorithms, and with powerful developer tools.
Using R.O.S. all programs (besides the modules) can become a node and is able to publish and get information from other nodes through topics which is the perfect way for these programs to communicate.

The Jetson Nano also controls all the sensors used in our project:
* `1 color sensor` which detects the orange and blue line
* `2 laser distance sensors` that are used to determine the distance to the nearest wall from the fron oh the vehicle.
* `1 gyroscope / accelerometer` that is constantly use to determine and control the orientation of the vehicle.
* `1 camera` which is used to detect what is in front of the vehicle

On the software part the following programs and modules are used:
   - `pillar.py`
   - `run1.py` contains the main program that will run during the first part of the competition using the modules mentioned below.
   - `run2.py` contains the main program that will run during the second part of the competition using the modules mentioned below.
   - `modules`
       - `cnf.txt` contains many variables used by the car.py that were found through contious testing, like the color codes of the lines etc.
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
       - `wall.py` contains the class used by the camera to manage and detect and avoid any walls that might be in front of the vehicle correctly imminent.

**run1.py**
This program runs the program for the first challenge. It runs on a loop. Inside the loop, there is a hierarchy of activities that are performed for each frame of the camera. Firstly, the program checks, using the front distance sensors, whether or not the vehicle is dangerously close to a wall. Then, based on that information, a pid algorithm is used to calculate the steering angle of the vehicle. Then, the vehicle turns to avoid the wall. Alternatively, if no walls are detected it checks, through the distance sensor, whether or not the distance on the left or the right is over 2m, which signifies a left or right turn respectively. If any such distance is detected, left or right, the vehicle performes a 90 degree turn to move into the next section of the field. As the program uses an algorithm for the steering direction to stay parrallel to the walls, which is done using an angle target on the compass, a turn can be effectively executed by simply incrementing or dicrementing the target of the algorith by 90 degrees. After it has completed all the turns it stops in the starting sector. 

**run2.py** 
This program runs the program for the second challenge. It has multiple similarities to the run 1 function that runs in the first challenge. The only notable difference is that the program proceeds to check for pillars after the wall check. After the detection of a pillar, if it is present, the program will turn right or left, if needed, according to the distance, size and color (red or green) of the pillar. 

**pillar.py**
This file contains the program that has the finctionality of detecting the pillars and their position.

**cnf.txt**
The text file where values are stored after calibration, in order for them to be accessed and used by the run1 and run2 programs. 

**Modules Folder**
This is a folder that contains the modules that are used by the car.py program and by its numerous functions. 

# Credits - Libraries

- **Grove I2C Bus**

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


 - **Grove I2C Color Sensor**
   
   Library for Grove - I2C Color Sensor V2(https://www.seeedstudio.com/Grove-I2C-Color-Sensor-V2-p-2890.html)
