#!/usr/bin/python3
from serial_communication import Esp32_Communication
from camera import Camera

from logging import DEBUG as logging_debug_level, basicConfig as logging_basic_config, error as logging_error, info as logging_info
from cv2 import imshow, waitKey
from datetime import datetime


"""
To prepare the jetson for the serial communication do the following:

git clone https://github.com/JetsonHacksNano/UARTDemo
cd UARTDemo
sudo chmod 666 /dev/ttyTHS1
sudo python3 uart_example.py
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

                frame = camera.get_frame()
                imshow("Camera", frame)
                key = waitKey(1) & 0xFF
                if key == 27:
                    break
            
            Esp32.close()

        except KeyboardInterrupt:
            logging_info("[" + str(datetime.now())[0:18] + "] Program closed due to: KeyboardInterupt (Ctrl + C)")
            break

        except Exception as exception_error:
            logging_error("[" + str(datetime.now())[0:18] + "] " + str(exception_error))

if __name__ == '__main__':
    main()
