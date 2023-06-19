#!/usr/bin/env python3
# license removed for brevity
import cv2, queue, threading, time, os

WIDTH = 640 
HEIGHT = 480
FRAMERATE = 30
FLIP_METHOD = 0

# bufferless VideoCapture
class Camera:

  def __init__(self):
    cmd = 'sudo service nvargus-daemon restart'
    os.system(cmd)
    time.sleep(3)
    streamer = ("nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                WIDTH,
                HEIGHT,
                FRAMERATE,
                FLIP_METHOD,
                WIDTH,
                HEIGHT,
            ))
    
    self.kill_camera = False
    self.cap = cv2.VideoCapture(streamer, cv2.CAP_GSTREAMER)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()
    

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while not self.kill_camera:

      ret, frame = self.cap.read()
      if not ret:
        raise Exception("Fail to capture image!")

      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)
    self.cap.release()

  def read(self):
    if self.q.empty():
      return None
    return self.q.get()
  
  def reboot(self):
    self.kill_camera = True
    time.sleep(0.2)
    # self.cap.release()
    # cmd = 'sudo chmod 666 /dev/ttyTHS1'
    # os.system(cmd)
    # self.__init__()


if __name__ == "__main__":
  cap = Camera()
  while True:
    time.sleep(.5)   # simulate time between events
    frame = cap.read()
    cv2.imshow("frame", frame)
    if chr(cv2.waitKey(1) & 255) == 'q':
      break