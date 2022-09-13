try:
    from modules.cvtool import CVTool
except ModuleNotFoundError:
    from cvtool import CVTool
from json import loads
from math import atan
import numpy as np
from math import atan2, pi
import time
import cv2
import os


def change_range(value, in_min, in_max, out_min, out_max):
    scaled_value = (value - in_min) * (out_max - out_min) / \
        (in_max - in_min) + out_min
    return int(scaled_value)


class Pillar:
    def __init__(self, cvtool, color, filename="cnf.txt"):
        self.cnf_file = filename
        # print(self.cnf_file)
        self.color = color
        self.hsv_lowers = None
        self.hsv_uppers = None
        self.min_area = 1000
        self.object_size = int(100 * 7.8)
        self.height_cm = 10
        self.turn_side = None
        self.safe_lateral_distance = 20
        self.safe_distance_from_object = 100
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
                if self.color in dict_line["data"]:
                    lowers.append(
                        [dict_line['hmin'], dict_line['smin'], dict_line['vmin']])
                    uppers.append(
                        [[dict_line['hmax'], dict_line['smax'], dict_line['vmax']]])
                    turn_side = dict_line['turn']

        self.hsv_lowers = lowers
        self.hsv_uppers = uppers
        self.turn_side = turn_side

    def detect(self, image, draw=False):
        '''
        looking for the pillar
        '''
        # print(self.color)
        window_size = image.shape[1]
        mask = self.cvtools.find_colors(
            image, self.hsv_lowers, self.hsv_uppers)
        img_contours, contours = self.cvtools.find_contours(
            image, mask, min_area=1000, sort=True, filter=0, draw=draw)
        if len(contours) > 0:
            area = contours[0][0]
            if area > self.min_area:
                cx = contours[0][1][0] + contours[0][1][2] // 2
                #cy = contours[0][1][1] + contours[0][1][3] // 2
                h = contours[0][1][3]
                distance_from_object, lateral_distance = self.d_l(
                    cx, h, window_size)
                # print("Dis from object: ", distance_from_object,
                #       ", lateral dis: ", lateral_distance)
                if distance_from_object < self.safe_distance_from_object:
                    direction = self.turn_side
                    steering = lateral_distance
                    if direction == -1:
                        steering = lateral_distance - 100
                    steering = max(-100, min(100, int(steering * (self.safe_distance_from_object/distance_from_object))))
                    return img_contours, [steering, area]
        return None

    # This function returns the distance and lateral distance of the signal from the car

    def d_l(self, sx, sy, window_size):
        y = sy * 0.0264583333  # Converting to cm
        # print("Y: ", str(y))
        distance_from_object = int(
            (self.cvtools.focal_distance * self.object_size) / y)
        distance_from_object = max(3, distance_from_object)
        # print(sx, window_size//2)
        # x = ((window_size // 2) - sx) * self.cvtools.px_to_cm  # Converting to cm
        x_per = change_range(sx, 0, window_size, 0, 100)
        # print("x_per", x_per)
        return distance_from_object, int(x_per)

    # # This function returns the distance and lateral distance of the signal from the car
    # def d_l(self, sx, sy, window_size):
    #     y = sy * 0.0264583333  # Converting to cm
    #     print("Y: ", str(y))
    #     distance_from_object = int(
    #         (self.cvtools.focal_distance * self.object_size) / y)
    #     distance_from_object = max(3, distance_from_object)
    #     x = ((window_size // 2) - sx) * \
    #         self.cvtools.px_to_cm  # Converting to cm
    #     print("X: ", str(x))
    #     lateral_distance = int(
    #         (x * distance_from_object * self.cvtools.focal_distance)/10)
    #     lateral_distance = max(3, lateral_distance)
    #     return distance_from_object, lateral_distance


def main():

    cvtool = CVTool(use_cam=False)
    green_pillar = Pillar(cvtool=cvtool, color="green")
    red_pillar = Pillar(cvtool=cvtool, color="red")
    pillars = [green_pillar, red_pillar]
    # red_pillar.print()
    files = os.listdir("26_08_2022_17_09_43")
    for f in files:
        image_path = os.path.join("26_08_2022_17_09_43", f)
        print(image_path)
        start_time = time.time() * 1000
        image = cv2.imread(image_path)
        biggest_pillar = None
        for pillar in pillars:
            detected_pillar = pillar.detect(image)
            if detected_pillar is not None:
                if biggest_pillar is None:
                    biggest_pillar = detected_pillar
                elif detected_pillar[1][1] > biggest_pillar[1][1]:
                    biggest_pillar = detected_pillar
        if biggest_pillar is not None:
            # steering = biggest_pillar[1][0] * biggest_pillar[1][1]
            print("Pillar detected, steering: " +
                  str(biggest_pillar[1][0]) + ", area: " + str(biggest_pillar[1][1]))
            cv2.imshow("Image", biggest_pillar[0])
        else:
            cv2.imshow("Image", image)
        cv2.waitKey(0)
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
