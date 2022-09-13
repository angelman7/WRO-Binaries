# MIT License
# Copyright (c) 2019 JetsonHacks
# See license
# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2
from time import sleep
# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

class Camera:
    def __init__(self, width=640, height=480, framerate=60, flip_method=0, rsp_cam=True):
        if rsp_cam:
            streamer = ("nvarguscamerasrc ! "
                        "video/x-raw(memory:NVMM), "
                        "width=(int)%d, height=(int)%d, "
                        "format=(string)NV12, framerate=(fraction)%d/1 ! "
                        "nvvidconv flip-method=%d ! "
                        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                        "videoconvert ! "
                        "video/x-raw, format=(string)BGR ! appsink"
                        % (
                            width,
                            height,
                            framerate,
                            flip_method,
                            width,
                            height,
                        ))
            self.cap = cap = cv2.VideoCapture(streamer, cv2.CAP_GSTREAMER)
        else:
            self.cap = cv2.VideoCapture(0)

    def read(self):
        ret_val, img = self.cap.read()
        if not ret_val:
            return None
        return img

    def is_open(self):
        return self.cap.isOpened()

def main():
    cam = Camera(rsp_cam=True)
    sleep(1)
    while True:
        if cam.is_open():
            img = cam.read()
            cv2.imshow("Camera", img)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

if __name__ == "__main__":
    main()
