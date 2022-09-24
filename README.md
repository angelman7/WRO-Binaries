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

_This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._
  
## The Hardware
The Hardware consinsts of a Jetson Nano (running Linux) thats is the "mind" of the robot as it is where all the code is run.

Concerning the movement of the vehicle, a DC and a servo motor are used. The DC motor is responsible for the speed of the car while the servo motor is responsible for the steering of the car.

The Jetson Nano also controls all the sensors used in our project:
* `1 color sensor` which detects the orange and blue lines
* `1 OLED screen` which displays some info about the vehicle
* `3 selection buttons` which control what is shown on the OLED screen
* `1 camera` which is used to detect what is in front of the vehicle
     
We are also using a Jetson Nano using the Linux OS, which is using all the input sensors mentioned above

## The Software
The software of the car is split into two parts:
   - car.py (the main program that will run during the competition using the modules mentioned below)
   - cnf.txt (that contains many variables used by the car.py that were found through contious testing, like the color codes of the lines etc.)
   - modules
       - bmm150.py
       - button.py
       - camera.py
       - color.py
       - compass.py
       - cvtool.py
       - distance.py
       - drive.py
       - grove_i2c_bus.py
       - grove_i2c_color_sesor_v2.py
       - line.py
       - line2turn.py
       - oled.py
       - oled_sample.py
       - pillar.py
       - wall.py
   
 **Car.py**
 [...]
 **Cnf.txt**
 [...]
 **Modules**
 [...]

## Credits - Libraries
