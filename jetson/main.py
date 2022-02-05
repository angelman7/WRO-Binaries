from camera import Camera
import cv2

def main():
    camera = Camera(type=0)
    while True:
        frame = camera.get_frame()
        cv2.imshow("camera class", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

if __name__ == '__main__':
    main()