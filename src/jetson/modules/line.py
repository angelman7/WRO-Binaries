try:
    from modules.cvtool import CVTool
except ModuleNotFoundError:
    from cvtool import CVTool
from json import loads
from math import atan
import numpy as np
from math import sqrt
import time
import cv2
import os



# colors 'blue' or 'orange'
class Line:
    def __init__(self, cvtool, color, filename="cnf.txt"):
        self.cnf_file = filename
        self.color = color
        self.hsv_lowers = None
        self.hsv_uppers = None
        self.min_area = 1000
        self.turn_side = None
        self.debug_mode = False
        self.read_cnf_file()
        self.cvtools = cvtool
        

    def print(self):
        print("Pillar color:", self.color)
        print("Lowers:", self.hsv_lowers)
        print("Uppers:", self.hsv_uppers)
        print("Turn side:", self.turn_side)

    def set_debug_mode(self, debug=False):
        self.debug_mode = debug

    def read_cnf_file(self):
        lowers = []
        uppers = []
        i = 0
        turn_side = 0
        # print("self.cnf_file:", self.cnf_file)
        with open(self.cnf_file, 'r') as f:
            for line in f:
                # print(line)
                dict_line = loads(line)
                if self.color + "_line" in dict_line["data"]:
                    lowers.append([dict_line['hmin'], dict_line['smin'], dict_line['vmin']])
                    uppers.append([[dict_line['hmax'], dict_line['smax'], dict_line['vmax']]])
                    turn_side = dict_line['turn']
                    
        self.hsv_lowers = lowers
        self.hsv_uppers = uppers
        self.turn_side = turn_side

    def detect(self, image, draw=False):
        '''
        looking for the line
        '''
        mask = self.cvtools.find_colors(image, self.hsv_lowers, self.hsv_uppers)
        img_contours, contours = self.cvtools.find_contours(image, mask, min_area=1000, sort=True, filter=0, draw=draw)
        if len(contours) > 0:
            area = contours[0][0]
            # print(area)
            if area > self.min_area:
                cx = contours[0][1][0] + contours[0][1][2] // 2
                cy = contours[0][1][1] + contours[0][1][3] // 2
                radius = int(sqrt(cx ** 2 + cy ** 2))
                
                img_contours = cv2.circle(img_contours, (cx, cy), radius, (255, 255, 255), 5)
                return img_contours, (cx, cy) 
        return None


def main():
    cvtool = CVTool(use_cam=False, rsp_cam=True)
    blue_line = Line(cvtool, "blue")
    orange_line = Line(cvtool, "orange")
    
    # red_pillar = Pillar(cvtool=cv
    # tool, color="red")
    
    blue_line.print()
    # red_pillar.print()
    img_folder = "19_07_2022_20_30_29"
    # img_folder = "26_08_2022_17_09_43"
    for f in os.listdir(img_folder):
        image_name = img_folder + "/" + f
        print(image_name)
        image = cv2.imread(image_name)
        line_detection = blue_line.detect(image, draw=False)
        blue_y = -1
        orange_y = -1
        if line_detection is not None:
            cv2.imshow("Blue", line_detection[0])
            cv2.waitKey(0)
            blue_y = line_detection[1][1]
        time.sleep(0.2)
        line_detection = orange_line.detect(image, draw=False)
        if line_detection is not None:
            cv2.imshow("Orange", line_detection[0])
            cv2.waitKey(0)
            orange_y = line_detection[1][1]
        if orange_y == -1 or blue_y == -1:
            print("UKNOWN!")
            cv2.imshow("Image", image)
            cv2.waitKey(0)            
        elif orange_y > blue_y:
            print("Orange first")
        else:
            print("Blue first")
        time.sleep(0.2)
        cv2.destroyAllWindows()
    

    # fps = 0
    # frame_counter = 0
    # start = time.time()
    # while True:
    #     if cvtool.camera_is_open():
    #         start_time = time.time()
    #         _, frame = cvtool.camera.read()
    #         pillar_detection = green_pillar.detect(image=frame, draw=True)
    #         if pillar_detection:
    #             cv2.imshow("Pillar", pillar_detection[0])
    #         else:
    #             cv2.imshow("Pillar", frame)
    #         frame_counter += 1
    #         end_time = time.time()
    #         fps = 1//(end_time - start_time)
    #         print("FPS :", fps)
    #         key = cv2.waitKey(1) & 0xFF
    #         if key == 27:
    #             end = time.time()
    #             break
    
    # print("Average FPS: ", frame_counter / (end - start))


if __name__ == "__main__":
    main()
