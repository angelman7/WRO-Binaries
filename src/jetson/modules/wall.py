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

class Wall:
    def __init__(self, cvtool, filename="cnf.txt"):
        self.cnf_file = filename
        self.hsv_lowers = None
        self.hsv_uppers = None
        self.read_cnf_file()
        self.line_angle_threshold=75
        self.visual_surface_width=48 
        self.visual_surface_height=87 
        self.visual_surface_distance=15
        self.least_lateral_distance = 60
        self.least_front_distance = 50
        self.cvtool = cvtool

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
                if dict_line["data"] == "wall":
                    lowers = [dict_line['hmin'], dict_line['smin'], dict_line['vmin']]
                    uppers = [dict_line['hmax'], dict_line['smax'], dict_line['vmax']]
                    break

        self.hsv_lowers = lowers
        self.hsv_uppers = uppers

    def warp_image(self, image, points, width, height):
        pts1 = np.float32(points)
        pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        image_warp = cv2.warpPerspective(image, matrix, (width, height))
        return image_warp

    # This function returns the points detected by the Hough line transform
    def find_lines(self, line_details):
        rho, theta = line_details[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        px1 = int(x0 + 1000 * -b)
        py1 = int(y0 + 1000 * a)
        px2 = int(x0 - 1000 * -b)
        py2 = int(y0 - 1000 * a)
        return px1, px2, py1, py2

    # This function converts the points of the Hough line transform to actual coordinates in the window 
    def get_real_coords(self, x1, x2, y1, y2, l, b):
        m = (y2 - y1) / (x2 - x1)
        coords = []
        coords.append(int((m*x1 - y1 + 0) / m))
        coords.append(int(y1 - m*(x1 - 0)))
        coords.append(int((m*x1 - y1 + b) / m))
        coords.append(int(y1 - m*(x1 - l)))
        for i in range(len(coords)):
            if i % 2 == 0:
                if coords[i] < 0:
                    coords[i] = 0
                if coords[i] > l:
                    coords[i] = 0
            else:
                if coords[i] < 0:
                    coords[i] = 0
                if coords[i] > b:
                    coords[i] = 0          
        rx1 = 0
        ry1 = 0
        rx2 = 0
        ry2 = 0 
        for i in range(len(coords)):
            if i == 0:
                if coords[i] != 0:
                    rx1 = coords[i]
                    ry1 = 1
            if i == 1:
                if coords[i] != 0:
                    if ry1 == 0:
                        rx1 = 1
                        ry1 = coords[i]
                    else:
                        rx2 = 1
                        ry2 = coords[i]
            if i == 2:
                if coords[i] != 0:
                    if rx1 == 0:
                        rx1 = coords[i]
                        ry1 = b
                    else:
                        rx2 = coords[i]
                        ry2 = b
            if i == 3:
                if coords[i] != 0:
                    if ry1 == 0:
                        rx1 = l
                        ry1 = coords[i]
                    else:
                        rx2 = l
                        ry2 = coords[i]
        return rx1, ry1, rx2, ry2

    def detect_wall(self, image, top_crop=100):
        threshold = 6
        width = image.shape[1]
        height = image.shape[0]
        points = np.float32([(10, top_crop), (width - 10, top_crop),
                         (0, height-10), (width - 0, height-10)])
        image = self.warp_image(image, points, width, height)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # print(self.hsv_uppers)
        lower_limit = np.array(self.hsv_lowers)
        upper_limit = np.array(self.hsv_uppers)
        mask1 = cv2.inRange(hsv_image, lower_limit, upper_limit)
        kernel = np.ones((7, 7), np.uint8)
        # cv2.imshow("Mask", mask1)
        # cv2.waitKey(0)
        mask1 = cv2.dilate(mask1, kernel, iterations=3)
        mask1 = cv2.erode(mask1, kernel, iterations=8)
        # cv2.imshow("erode", mask1)
        # cv2.waitKey(0)
        
        image_2 = cv2.bitwise_and(image,image,mask = mask1)
        edges = cv2.Canny(image_2, 100, 200)
        lines = cv2.HoughLines(edges,1,np.pi/180, 50)
        right_wall = 0
        left_wall = 0
        front_wall = 0
        line_slopes = []
        fx1, fx2, fy1, fy2, rx1, rx2, ry1, ry2, lx1, lx2, ly1, ly2 = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        
        if lines is not None:
            for line in lines:
                try:
                    x1, x2, y1, y2 = self.find_lines(line)
                    angle = 180/np.pi*atan(1/((y2 - y1) / (x2 - x1)))
                    # print("angle:", angle)
                    if round(angle) not in line_slopes and not (-10 < angle < 10):
                        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        line_slopes.append(round(angle))
                        if angle > self.line_angle_threshold or angle < -self.line_angle_threshold:
                            fx1, fx2, fy1, fy2 = x1, x2, y1, y2
                            front_wall = angle
                            # print("Front wall")
                        elif angle > 0:
                            rx1, rx2, ry1, ry2 = x1, x2, y1, y2
                            right_wall = angle
                            # print("Right wall")
                        else:
                            lx1, lx2, ly1, ly2 = x1, x2, y1, y2
                            left_wall = angle
                            # print("Left wall")
                        # cv2.imshow("line", image)
                        # cv2.waitKey(0)
                except:
                    pass
        
        if left_wall != 0 - threshold and left_wall != 0:
            llx1, lly1, llx2, lly2 = self.get_real_coords(lx1, lx2, ly1, ly2, width, height)
            P1_disy = self.visual_surface_height * (1-(lly1/height)) + self.visual_surface_distance
            P2_disy = self.visual_surface_height * (1-(lly2/height)) + self.visual_surface_distance
            P1_disx = self.visual_surface_width * (llx1/width)
            P2_disx = self.visual_surface_width * (llx2/width)
            try:
                m = (P1_disy - P2_disy) / (P1_disx - P2_disx)
            except ZeroDivisionError:
                m = 1
            line_dis = (m * P2_disx - P2_disy + 0) / m
            # print("Left line distance: ", line_dis)
            if abs(line_dis) < self.least_lateral_distance:
                return image, "R", 100 - abs(left_wall)
            
        if right_wall != threshold and right_wall != 0:
            llx1, lly1, llx2, lly2 = self.get_real_coords(rx1, rx2, ry1, ry2, width, height)
            P1_disy = self.visual_surface_height * (1-(lly1/height)) + self.visual_surface_distance
            P2_disy = self.visual_surface_height * (1-(lly2/height)) + self.visual_surface_distance
            P1_disx = self.visual_surface_width * (llx1/width)
            P2_disx = self.visual_surface_width * (llx2/width)
            m = (P1_disy - P2_disy) / (P1_disx - P2_disx)
            line_dis = ((m * P1_disx - P1_disy + 0) / m) - 15
            # print("Right line distance: ", line_dis)
            if abs(line_dis) < self.least_lateral_distance:
                return image, "L", abs(right_wall) - 100
            
        if front_wall != 0:
            llx1, lly1, llx2, lly2 = self.get_real_coords(fx1, fx2, fy1, fy2, width, height)
            

            P1_dis = self.visual_surface_width * (1-(lly1/width)) + self.visual_surface_distance
            P2_dis = self.visual_surface_width * (1-(lly2/width)) + self.visual_surface_distance
            line_dis = (P1_dis + P2_dis)/2
            # print("Front line distance: ", line_dis)
            if line_dis < self.least_front_distance:
                return image, "F",  abs(front_wall)
            
        return image, "N", 0

    def camera_test(self):
        fps = 0
        frame_counter = 0
        start = time.time()
        order = 0
        while True:
            if self.cvtool.camera_is_open():
                start_time = time.time()
                suc, frame = self.cvtool.camera.read()
                if suc:
                    wall_image, dir, angle = self.detect_wall(frame)
                    cv2.imshow("image", frame)
                    cv2.imshow("wall_image", wall_image)
                    print(dir, angle)
                    cv2.waitKey(1)
        

def main():
    cvtool = CVTool(use_cam=False, rsp_cam=True)
    wall = Wall(cvtool=cvtool)
    for i in range(1, 400):
        image_name = f"19_07_2022_20_30_29/img_{str(i+1)}.png"
        image = cv2.imread(image_name)
        wall_image, dir, angle = wall.detect_wall(image)
        cv2.imshow("image", image)
        cv2.imshow("wall_image", wall_image)
        print(image_name, dir, angle)
        cv2.waitKey(0)
    # wall.camera_test()


if __name__ == "__main__":
    main()


