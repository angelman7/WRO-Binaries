import cv2
import numpy as np
from pillar import Pillar
from camera import Camera


class PillarDetector:
    def __init__(self, pillars):
        self.pillars = pillars

    def detect_pillars(self, image):
        detected = []
        # type your code

    def find_color(self, image, hsv_image, hsv_bounds):
        """
        :param img: Image in which color needs to be found
        :param hsvVals: List of lower and upper hsv range
        :return: (mask) bw image with white regions where color is detected
                 (imgColor) colored image only showing regions detected
        """

        mask = None
        for bound in hsv_bounds:
            lower = np.array(bound[0])
            upper = np.array(bound[1])
            # print(lower)
            # print(upper)
            if mask is None:
                mask = cv2.inRange(hsv_image, lower, upper)
            else:
                mask |= cv2.inRange(hsv_image, lower, upper)
        image_color = cv2.bitwise_and(image, image, mask=mask)
        # cv2.imshow("Contours", image_color)
        # cv2.waitKey(1)
        return image_color

    def pre_processing(self, image, blur=5, canny_thres=None, dia=1):
        """
        Preprocessing of the image to find edges
        :param img: Image to preprocess
        :param blur: blur kernel size, must be odd number
        :param cannyThresh: Canny threshold Values
        :param dia: Dilation itteration value
        :return: preprocessed image showing the edges
        """
        if canny_thres is None:
            canny_thres = [50, 50]
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_blur = cv2.GaussianBlur(image_gray, (blur, blur), 1)
        image_canny = cv2.Canny(image_blur, canny_thres[0], canny_thres[1])
        kernel = np.ones((5, 5), np.uint8)
        img_dia = cv2.dilate(image_canny, kernel, iterations=dia)
        return img_dia

    def find_contours(self, image, img_pre, min_area=10, sort=True, filter=0, draw_con=True):
        """
        Finds Contours in an image
        :param img: Image on which we want to draw
        :param imgPre: Image on which we want to find contours
        :param minArea: Minimum Area to detect as valid contour
        :param sort: True will sort the contours by area (biggest first)
        :param filter: Filters based on the corner points e.g. 4 = Rectangle or square
        :param drawCon: draw contours boolean
        :return: Foudn contours with [contours, Area, BoundingBox]
        """
        con_found = []
        img_contours = image.copy()
        # cv2.CHAIN_APPROX_NONE: all the boundary points are stored
        # cv2.CHAIN_APPROX_SIMPLE: keep only points we need (2 points for a line)
        contours, hierarchy = cv2.findContours(img_pre, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                # print(len(approx))
                if len(approx) == filter or filter == 0:
                    if draw_con: cv2.drawContours(img_contours, cnt, -1, (255, 0, 255), 3)
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(img_contours, (x + (w // 2), y + (h // 2)), 5, (0, 255, 0), cv2.FILLED)
                    con_found.append([cnt, area, [x, y, w, h]])

        if sort:
            con_found = sorted(con_found, key=lambda x: x[1], reverse=True)
        return img_contours, con_found


