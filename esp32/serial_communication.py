from machine import UART
from time import sleep

class Jetson_Communication:
    def __init__(self, baudrate=115200) -> None:
        self.uart = UART(2, baudrate)

    def read(self):
        if self.uart.any():
            # read or readline
            message = self.uart.readline()
            return message

    def send(self, message):
        self.uart.write(message)

esp = Jetson_Communication()
while True:
    e = esp.read()
    if not (e is None):
        print(e.decode())
        esp.send(e)
    sleep(1)
