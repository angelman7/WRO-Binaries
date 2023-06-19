import cv2
import numpy as np
import modules.hsv_thresholds as hsv_thr
import modules.contour_detection as con 
import modules.color_detection as col
from time import sleep
import os

current_directory = os.getcwd()
images_directory = os.path.join(current_directory, "images")
img_file = None
print("Images directory:", images_directory)
for img_file in os.listdir(images_directory):
    if img_file.endswith(".png"):
        img_file = os.path.join(images_directory, img_file)
        print("image file:", img_file)
        image = cv2.imread(img_file)
        cv2.imshow("Image",image)
        print("Crop? (y/n)")
        key = cv2.waitKey(0) & 0xFF
        if key == ord('n') or key == ord('N'):
            continue
        elif key == ord('y') or key == ord('Y'):                    
            image, lower, upper = hsv_thr.get_thresholds(image)
            print(lower, upper)
            mask, color_image = col.findColor(image, lower, upper)
            imgPre = con.preProcessing(color_image)
            imgContours, conFound = con.findContours(color_image, imgPre, minArea=500)
            cv2.imshow("Image",color_image)
            cv2.imshow("Image Contours",imgContours)
            cv2.waitKey(0)
            sleep(1)