from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial
from time import sleep
from random import randint
import subprocess
import sys

"""
To prepare the jetson for the serial communication do the following:
git clone https://github.com/JetsonHacksNano/UARTDemo
cd UARTDemo
sudo chmod 666 /dev/ttyTHS1
sudo python3 uart_example.py
"""


class Esp32_Communication:
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE) -> None:
        subprocess.run(["sh", "./port.sh"])
        self.serial_port = Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
        )
        # Wait a second to let the port initialize
        sleep(1)

    def read(self):
        if self.serial_port.inWaiting() > 0:
            return self.serial_port.readline().decode()

    def send(self, message) -> None:
        try:
            self.serial_port.write(message.encode())
            if message == "\r".encode():
                self.serial_port.write("\n".encode())
            return True
        except:
            return False
    
    def close(self):
        self.serial_port.close()

esp = Esp32_Communication()
while True:
    suc = esp.send(str(randint(0, 115)) + "\n") # sends angle for esp32 servo to turn
    if suc:
        print("Message succesfully sent!")
    else:
        print("Can't send message!")
                                         
    message = esp.read()
    if message is None:
        pass
    else:
        print("Message received:", message, end="")
    sleep(10)

