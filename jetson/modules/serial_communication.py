from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial
from time import sleep


"""
To prepare the jetson for the serial communication do the following:

git clone https://github.com/JetsonHacksNano/UARTDemo
cd UARTDemo
sudo chmod 666 /dev/ttyTHS1
sudo python3 uart_example.py
"""


class Esp32_Communication:
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE) -> None:
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
