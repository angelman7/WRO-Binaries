# WRO - NameNotFound

This is the project for the WRO 2022 Contest: Future Engineers
The jetson folder contains the code that is currently in the jetson
The ESP32 folder contains the code that is currently in the ESP32

Project ToDo list: (Trello)
  - https://trello.com/b/YYgFc8AK/future-engeneers-project

## The Hardware
At the start we used an EV3 Lego Mindstorms Brick to check the speed of the car and whether or not it was drivable.

Then, concerning the control of the motors (DC motor that is responsible for the speed of the car, servo motor that is responsible for the steering of the car), we changed to ESP32 a microcontroller. The ESP32 also controls all the sensors and other tools used in our project (1 color sensor, 2 ultrasonic sensors, 1 OLED screen, 3 selection buttons, 1 potentiometer). We are also using a Jetson Nano using the Linux OS, which is responsible for the camera and processing its frames. The microcontroller and the computer communicate through UART Serial communication, which in terms of hardware is done connecting each unit's TX port with the other unit's RX port and vice versa.

## The Software
The software of the car is split into two parts:
 - ESP32 programs (split into 3 parts due to the ESP32 limitations) which contains the programs:
   - main programs:
      * configuration.py
      * run1.py
      * run2.py
   - modules:
      * ssd1306.py
      * hscr04.py
 
 - Jetson programs
   - main programs:
      * car_control.py (the main program that will run during the competition)
      * configuration.py
   - modules:
     * serial_communication.py (module that established UART serial communication between the ESP32 and the Jetson Nano)
     * camera.py
     * line_following.py
     * cvtools.py
     * object_following
     
     * to be continued


**ESP32 PROGRAMS**
1) The Configuration file:
configures the basic settings and values needed for the color sensor to track the game mat's orange and blue lines correctly, the hsv values of the pillars, the left and right value limits of the servo motor, and the digital compass' values.
#   - Firstly, the program can configure the compass by asking the user to configure the robot in the four directions of the level (Forward, Backward, Left, Right). #These will help the run program know its position later when it will run.
   
2) The Run 1 file:
  dadadadadajjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
  
3) The Run 2 file:
  dadadafafafdhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
   
 **JETSON NANO PROGRAMS**
 1) The Configuration file:
The program can configure the colors of the pillars that must be detected. This is done by asking the user to take photos of the desired pillar. Then the user selects the pillar from the image by selecting a rectangle of the pillar with the specific color. The program selects the upper and lower values of color (H min, H max, S min, S max, V min, V max) it has selected(These can be changed by the user using the 6 trackbars that appear on a new window). There will also be a second window that will show what the image contains within these margins(upper and lower).
Finaly, the program will save the values into a seperate text file for them to be used by the other programs




## Credits - Libraries
