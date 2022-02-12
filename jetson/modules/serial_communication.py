from time import sleep
import serial

class Esp32_Communication:
    def __init__(self, port="/dev/ttyTHS1", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) -> None:
        self.serial_port = serial.Serial(
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
            data = self.serial_port.read()
            return data

    def send(self, message) -> None:
        self.serial_port.write(message)
        if message == "\r".encode():
            self.serial_port.write("\n".encode())