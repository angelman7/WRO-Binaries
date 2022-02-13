from datetime import datetime
import logging

"""
The program that will run on the Jetson Nano 2GB during the 2nd phase of the competition.
It will detect pillars and report to the 2nd system (ESP32) via serial communication 
(to investigate whether the messages can be exchanged as json data and whether this method
affects the speed). The detection will be done with HSV analysis in specific value ranges
(lower & upper) of the pillars. Lower & upper values will be provided either by default
or by file. The final product of the scan, which will be sent to ESP32 serially (eg 
jason data, string etc), will include:
 - flag that will indicate positive or negative detection
 - nearest pillar species (green / red)
 - distance of the nearest pillar from the camera (estimate)
 - position of the closest pillar to the center of the screen (horizontal and / or vertical)
"""

logging.basicConfig(filename="./jetson.log", level=logging.DEBUG, encoding='utf-8')

try:
    while True:
        pass
except Exception as e:
    logging.error(str(datetime.now())[0:18] + str(Exception))