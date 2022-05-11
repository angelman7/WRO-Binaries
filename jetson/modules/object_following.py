"""
Grant Permission to the port 
sudo chmod 666 /dev/ttyACM0

Change the mode to 5W for using with USB power
sudo nvpmodel -m1
sudo nvpmodel -q

by: Murtaza Hassan
Website: www.murtazahassan.com
Youtube: Murtaza's Workshop - Robotics and AI
"""

import cv2
import numpy as np 
import ColorModule as cm 
import ContourModule as cnm 
import SerialModule as sm 


ser = sm.initConnection('/dev/ttyACM0', 9600)

frameWidth = 360
frameHeight= 240
flip =  0
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(frameWidth)+', height='+str(frameHeight)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cap = cv2.VideoCapture(camSet)
hsvVals = [[0, 0, 0], [179, 56, 255]]
perror = 0

def trackObject(img, conFound, pid, perror):
    if len(conFound)!= 0:
        x,y,w,h = conFound[0][2]
        cx = x+(w//2)
        cy = y+(h//2)

        hi, wi, ci = img.shape
        cv2.line(img, (wi//2,cy),(cx,cy),(0,0,255),2)

        ### Pid 
        error = wi//2 - cx
        posX = int(pid[0]*error + pid[1]*(error-perror))
        posX = int(np.interp(posX,[-w//4,w//4],[-40,40]))
        perror = error
        #print(posX)
        sm.sendData(ser, [37,posX], 4)
    else:
        sm.sendData(ser, [0,0], 4)

    return img, perror

while True:
    success, img = cap.read()
    mask, imgColor = cm.findColor(img, hsvVals)
    imgContours, conFound = cnm.findContours(img, mask, 2500, drawCon= True)

    h, w, c = imgContours.shape
    cv2.line(imgContours,(w//2,0),(w//2,h),(0,0,255),2)

    imgContours, perror = trackObject(imgContours, conFound, [1/4,1/6],perror)

    #cv2.imshow("Image",img)
    #cv2.imshow("Image Color",imgColor)
    #cv2.imshow("Image Contours",imgContours)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
