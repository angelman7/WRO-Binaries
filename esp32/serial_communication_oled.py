from machine import UART, Pin, I2C
import ssd1306
from time import sleep

PIN_SCL = 22
PIN_SDA = 21

i2c = I2C(-1, scl=Pin(PIN_SCL),sda=Pin(PIN_SDA))
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

class Jetson_Communication:
    def __init__(self, baudrate=115200):
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
    print(e)
    if not (e is None):
        print(e.decode(), end='')
        oled.text(str(e.decode().strip('\n')), 0, 0)
        oled.show()
        esp.send(e)
    sleep(1)
