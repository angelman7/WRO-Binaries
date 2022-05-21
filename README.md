# WRO - NameNotFound

This is the project for the WRO 2022 Contest: Future Engineers
The jetson folder contains the code that is currently in the jetson
The ESP32 folder contains the code that is currently in the ESP32

Project ToDo list: (Trello)
  - https://trello.com/b/YYgFc8AK/future-engeneers-project

# The Hardware
At the start we used an EV3 Lego Mindstorms Brick to check the speed of the car and whether or not it was drivable.

# The Software
The software of the car is split into two parts:
 - ESP32 programs(which contains the programs "configuration.py", "run1.py" and "run2.py"
 - Jetson program(which contains a program which ...)


ESP32 PROGRAMS
1) The Configuration program configures the basic settings and values needed for the color sensor correctly & the hsv values of the pillars.
   - Firstly, the program can configure the compass by asking the user to configure the robot in the four directions of the level (Forward, Backward, Left, Right). These will help the run program know its position later when it will run.
   - Furthermore, the program can configure the colors of the pillars that must be detected. This is done by asking the user to take photos of the disired pillar. Then the user selects the pillar from the image by selecting a rectangle of the pillar with the specific color. The program selects the upper and lower values of color (H min, H max, S min, S max, V min, V max) it has selected(These can be changed by the user using the 6 trackbars that appear on a new window. There will also be a second window that will show what the image contains within these margins(upper and lower).
   - Finaly the program will save the values into a seperate text file for them to be used by the other programs

2) The Run 1 file:
  dadadadadajjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
4) The Run 2 file:
  dadadafafafdhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh

# Credits - Libraries
