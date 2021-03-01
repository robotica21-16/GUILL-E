import cv2
import picamera
from picamera.array import PiRGBArray
import numpy as np
import time

ESC = 27
KNN = 1
MOG2 = 2

cam = picamera.PiCamera()

cam.resolution = (320, 240)
#cam.resolution = (640, 480)
cam.framerate = 32
rawCapture = PiRGBArray(cam, size=(320, 240))
#rawCapture = PiRGBArray(cam, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)


for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = img.array
    cv2.imshow('Captura', frame)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    k = cv2.waitKey(1) & 0xff
    if k == ESC:
        cam.close()
        break

cv2.destroyAllWindows()

