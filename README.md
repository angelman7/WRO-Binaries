# WRO - NameNotFound

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2022.

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction

_This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._
  
## The Hardware
At the start we used an EV3 Lego Mindstorms Brick to check the speed of the car and whether or not it was drivable.

Then, concerning the control of the motors (DC motor that is responsible for the speed of the car, servo motor that is responsible for the steering of the car), we changed to ESP32 a microcontroller. The ESP32 also controls all the sensors and other tools used in our project (1 color sensor, 2 ultrasonic sensors, 1 OLED screen, 3 selection buttons, 1 potentiometer). We are also using a Jetson Nano using the Linux OS, which is responsible for the camera and processing its frames. The microcontroller and the computer communicate through UART Serial communication, which in terms of hardware is done connecting each unit's TX port with the other unit's RX port and vice versa.

## The Software
The software of the car is split into two parts:
 1) ESP32 programs (split into 3 programs due to the ESP32 limitations) which contains the programs:
     - configuration.py
     - run1.py
     - run2.py
     - modules
         - ssd1306.py
         - hscr04.py
 
 2) Jetson programs
     - car_control.py (the main program that will run during the competition)
     - configuration.py
     - modules    
         - serial_communication.py (module that established UART serial communication between the ESP32 and the Jetson Nano)
         - camera.py
         - line_following.py
         - cvtools.py
         - object_following
     
     _to be continued_


**ESP32 PROGRAMS**
1) The Configuration file:
configures the basic settings and values needed for the color sensor to track the game mat's orange and blue lines correctly, the hsv values of the pillars, the left and right value limits of the servo motor, and the digital compass' values.
- Firstly, the program can configure the compass by asking the user to configure the robot in the four directions of the level (Forward, Backward, Left, Right). #These will help the run program know its position later when it will run.
   
2) The Run 1 file:
  TBA
  
3) The Run 2 file:
  TBA
   
 **JETSON NANO PROGRAMS**
 1) The Configuration file:
The program can configure the colors of the pillars that must be detected. This is done by asking the user to take photos of the desired pillar. Then the user selects the pillar from the image by selecting a rectangle of the pillar with the specific color. The program selects the upper and lower values of color (H min, H max, S min, S max, V min, V max) it has selected(These can be changed by the user using the 6 trackbars that appear on a new window). There will also be a second window that will show what the image contains within these margins(upper and lower).
Finaly, the program will save the values into a seperate text file for them to be used by the other programs




## Credits - Libraries
