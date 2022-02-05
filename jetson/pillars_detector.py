import cv2
import numpy as np
from pillar import Pillar
from camera import Camera

class PillarDetector:
    def __init__(self, pillars) -> None:
        self.pillars = pillars
    
    def add_pillar(self, pillar):
        self.pillars.append(pillar)
    
    def detect_pillars(self, image):
        return detected_pillars
    
    def findColor(self, img, bounds):
        """
        :param img: Image in which color needs to be found
        :param hsvVals: List of lower and upper hsv range 
        :return: (mask) bw image with white regions where color is detected
                (imgColor) colored image only showing regions detected
        """
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        all_masks = None
        for bound in bounds:
            lower = np.array(bound[0])
            upper = np.array(bound[1])
            mask = cv2.inRange(imgHSV, lower, upper)
            if all_masks is None:
                all_masks = mask
            else:    
                all_masks |= mask
        img_color = cv2.bitwise_and(img, img, mask=all_masks)
        return mask, img_color