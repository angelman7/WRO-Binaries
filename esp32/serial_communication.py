from machine import UART

class Jetson_Communication:
    def __init__(self, baudrate=115200) -> None:
        self.uart = UART(2, baudrate)
    
    def read(self):
        if self.uart.any():
            message = self.uart.read().decode()
            return message
    
    def send(self, message):
        self.uart.write(message.encode())
