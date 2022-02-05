import logging
import cv2
import imutils

class Camera:
    def __init__(self, type=-1, width=640, height=480, framerate=90, flip=0):
        self.type = type
        self.width = width
        self.height = height
        self.framerate = framerate
        self.flip = flip

        try:
            if self.type == -1:
                self.camera = cv2.VideoCapture(self.gstreamer_pipeline())
            else:
                self.camera = cv2.VideoCapture(type)
        except Exception as exc:
            logging.error("ERROR: Cannot Initialize Camera with type" + str(type))

    def gstreamer_pipeline(self):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                self.width, 
                self.height, 
                self.framerate, 
                self.flip, 
                self.width, 
                self.height
            )
        )
    
    def get_frame(self):
        suc, frame = self.camera.read()
        if suc:
            return frame # imutils.resize(frame, self.width)
        logging.error("ERROR: Failed to capture image")