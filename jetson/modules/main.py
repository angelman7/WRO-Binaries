#!/usr/bin/python3
from serial_communication import communicate_with_esp32
from camera import Camera
import serial
import time
import cv2

# To prepare the jetson for the serial communication do the following:
#
# git clone https://github.com/JetsonHacksNano/UARTDemo
# cd UARTDemo
# sudo chmod 666 /dev/ttyTHS1
# sudo python3 uart_example.py

def main():
    try:
        print("UART Demonstration Program")
        print("NVIDIA Jetson Nano Developer Kit")

        serial_port = serial.Serial(
            port="/dev/ttyTHS1",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        # Wait a second to let the port initialize
        time.sleep(1)

        camera = Camera(type=0)

        while True:
            data = communicate_with_esp32()
                

            frame = camera.get_frame()
            cv2.imshow("camera class", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
        
        serial_port.close()

    except KeyboardInterrupt:
        print("Exiting Program")

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

if __name__ == '__main__':
    main()




