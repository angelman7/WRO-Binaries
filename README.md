# WRO---2022---Jetson

This is the project for the WRO 2022 Contest: Future Engineers
The jetson folder contains the code that is currently in the jetson
The ESP32 folder contains the code that is currently in the ESP32

Project ToDo list: (Trello)
  - https://trello.com/b/YYgFc8AK/future-engeneers-project

# The Hardware
At the start we used an EV3 Lego Mindstorms Brick to check the speed of the car and whether or not it was drivable.
Then Jetson. Then it was ready

# The Software
The software is split into 2 parts:
 - The ESP32 (which contains the programs "configuration.py", "run1.py" and "run2.py"
 - The Jetson (which contains a program which ...)

1) The Configuration program configures the basic settings and values needed for the color sensor correctly & t
he hsv values of the pillars.
 - Firstly, the program can configure the compass by asking the user to configure the robot in the four directions of the level (Forward, Backward, Left, Right). These will help the run program know its position later when it will run.
 - Furthermore, the program can configure the colors of the pillars that must be detected. This is done by asking the user to take photos of the disired pillar. Then the user selects the pillar from the image by selecting a rectangle of the pillar with the specific color. The program selects the upper and lower values of color (H min, H max, S min, S max, V min, V max) it has selected(These can be changed by the user using the 6 trackbars that appear on a new window. There will also be a second window that will show what the image contains within these margins(upper and lower).
Finaly the program will save the values into a seperate text file for them to be used by the other programs

# Credits - Libraries
