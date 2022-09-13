import Adafruit_SSD1306
import time
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

class OLED:
    def __init__(self):
        # 128x32 display with hardware I2C:
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1) # setting gpio to 1 is hack to avoid platform detection

        # Initialize library.
        self.disp.begin()

        # Clear display.
        self.disp.clear()
        self.disp.display()

        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))

        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)
        # Load default font.
        self.font = ImageFont.load_default() #ImageFont.truetype("NotoSansJP-Regular.ttf", size=20)

    def show_text_list(self, txt_list, selected=-1):
        self.clear()
        for i, txt in enumerate(txt_list):
            if i > 7:
                break
            if i == selected:
                txt = "> " + txt
            else:
               txt = "  " + txt
            self.draw.text((0, i * 8), txt, font=self.font, fill=255)
        self.disp.image(self.image)
        self.disp.display()
        # time.sleep(0.1)

    def show_text(self, text):
        self.show_text_list([text], selected=-1)

    def clear(self):
        self.image = Image.new('1', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)
        self.disp.image(self.image)
        self.disp.clear()
        self.disp.display()
        

def main():
    oled = OLED()
    oled.show_text("One Line")
    time.sleep(5)
    oled.clear()
    txt_list = ["Line No 1", "Line No 2", "Line No 3", 
                "Line No 4", "12345678912345678912", "Line No 7",
                "Line No 8"]
    oled.show_text_list(txt_list, selected=0)
    time.sleep(5)
    oled.clear()
    
    oled.show_text_list(txt_list, selected=4)
    time.sleep(5)
    

if __name__ == "__main__":
    main()
