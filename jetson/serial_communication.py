class Esp32_Communication:
    def __init__(self, serial_port) -> None:
        self.serial_port = serial_port

    def read(self):
        if self.serial_port.inWaiting() > 0:
            data = self.serial_port.read()
            return data

    def send(self, message) -> None:
        self.serial_port.write(message)
        if message == "\r".encode():
            self.serial_port.write("\n".encode())