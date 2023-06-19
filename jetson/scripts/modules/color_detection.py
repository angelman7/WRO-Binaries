"""
Color Module 
Finds color in an image based on hsv values
Can run as stand alone to find relevant hsv values
by: ROBOTONIO
Website: www.robotonio.gr
"""
import cv2
import numpy as np
from numpy.core.defchararray import lower 
import os

lower = [0, 0, 0]
upper = [179, 255, 255]
def empty(a):
    lower[0] = cv2.getTrackbarPos("Hue Min", "TrackBars")
    lower[1] = cv2.getTrackbarPos("Sat Min", "TrackBars")
    lower[2] = cv2.getTrackbarPos("Val Min", "TrackBars")
    upper[0] = cv2.getTrackbarPos("Hue Max", "TrackBars")
    upper[1] = cv2.getTrackbarPos("Sat Max", "TrackBars")
    upper[2] = cv2.getTrackbarPos("Val Max", "TrackBars")

def initTrackbars():
    """
    To intialize Trackbars . Need to run only once
    """
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars",640,240)
    cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
    cv2.createTrackbar("Hue Max","TrackBars",179,179,empty)
    cv2.createTrackbar("Sat Min","TrackBars",0,255,empty)
    cv2.createTrackbar("Sat Max","TrackBars",255,255,empty)
    cv2.createTrackbar("Val Min","TrackBars",0,255,empty)
    cv2.createTrackbar("Val Max","TrackBars",255,255,empty)

def findColor(img, lower, upper):
    """
    :param img: Image in which color needs to be found
    :param hsvVals: List of lower and upper hsv range 
    :return: (mask) bw image with white regions where color is detected
             (imgColor) colored image only showing regions detected
    """
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array(lower)
    upper = np.array(upper)
    mask = cv2.inRange(imgHSV, lower, upper)
    img_color = cv2.bitwise_and(img, img, mask=mask)
    return mask, img_color


def main():
    global lower, upper 

    folder = os.path.join("..", os.getcwd(), "images")

    img = cv2.imread(os.path.join(folder,"img_0.png"))
    mask, imgColor =  findColor(img, lower, upper)
    cv2.imshow("Image",img)
    cv2.imshow("TrackBars",imgColor)

if __name__ == "__main__":
    initTrackbars()
    while True:
        main()
        if cv2.waitKey(1) & 0xFF == 27:
            break