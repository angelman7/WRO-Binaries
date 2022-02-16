#!/usr/bin/python3
from modules.serial_communication import Esp32_Communication
from modules.camera import Camera

from logging import DEBUG as logging_debug_level, basicConfig as logging_basic_config, error as logging_error, info as logging_info
from cv2 import imshow, waitKey
from datetime import datetime


"""
The program that will run for the preparation of the 1st system, in the conditions
of the 1st part of the competition. With the current design, the file exists
for possible future development and utilization
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
                    raise KeyError("Program closed due to: Escape key pressed (X)")
                    logging_info("[" + str(datetime.now())[0:18] + "] Program closed due to: Escape key pressed (Esc)")
            
            Esp32.close()

        except KeyboardInterrupt:
            logging_info("[" + str(datetime.now())[0:18] + "] Program closed due to: KeyboardInterupt (Ctrl + C)")
            break

        except KeyError as exception_error:
            logging_info("[" + str(datetime.now())[0:18] + "] " + str(exception_error))
            break

        except Exception as exception_error:
            logging_error("[" + str(datetime.now())[0:18] + "] " + str(exception_error))

if __name__ == '__main__':
    main()
