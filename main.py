from camera import Camera
import cv2

def main():
    camera = Camera(type=0)
    while True:
        frame = camera.get_frame()
        cv2.imshow()