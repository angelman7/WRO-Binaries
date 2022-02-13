from modules.serial_communication import Esp32_Communication
from modules.camera import Camera

from logging import DEBUG as logging_debug_level, basicConfig as logging_basic_config, error as logging_error, info as logging_info
from cv2 import imshow, waitKey
from datetime import datetime


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


logging_basic_config(filename="./jetson.log", level=logging_debug_level, encoding='utf-8')

def main():
    while True:
        try:
            print("UART Demonstration Program")
            print("NVIDIA Jetson Nano Developer Kit")

            Esp32 = Esp32_Communication()

            camera = Camera(type=0)

            while True:
                data = Esp32.read()
                if not (data is None):
                    logging_info("[" + str(datetime.now())[0:18] + "] Message from Esp32: " + str(data))

                frame = camera.get_frame()
                imshow("Camera", frame)
                key = waitKey(1) & 0xFF
                if key == 27:
                    raise KeyError("Program closed due to: Escape key pressed (Esc)")
                    logging_info("[" + str(datetime.now())[0:18] + "] Program closed due to: Escape key pressed (Esc)")
            
            Esp32.close()

        except KeyboardInterrupt:
            logging_info("[" + str(datetime.now())[0:18] + "] Program closed due to: KeyboardInterupt (Ctrl + C)")
            break

        except KeyError as exception_error:
            logging_info("[" + str(datetime.now())[0:18] + "] " + str(exception_error))

        except Exception as exception_error:
            logging_error("[" + str(datetime.now())[0:18] + "] " + str(exception_error))

if __name__ == '__main__':
    main()
