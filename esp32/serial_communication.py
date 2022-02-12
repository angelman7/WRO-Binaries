from machine import UART

class Jetson:
    def __init__(self, uart_num) -> None:
        self.uart = UART(2, uart_num)
    
    def read(self):
        if self.uart.any():
            message = self.uart.read().decode()
            return message
    
    def send(self, message):
        self.uart.write(message.encode())
