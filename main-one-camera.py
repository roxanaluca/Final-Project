import cv2
from camera import Camera
import numpy as np
import RPi.GPIO as gp
import time

gp.setwarnings(False)
gp.setmode(gp.BOARD)
gp.setup(7,gp.OUT)
gp.setup(11,gp.OUT)
gp.setup(12,gp.OUT)

cv2.namedWindow("eyecameras")
camera = Camera()
camera.start()
time.sleep(0.1)

while camera.hasSignal():
   key = cv2.waitKey(40)
   if key == 27: # exit on ESC
      break
   framequeue1 = camera.getPicture()
   for i in range(5):
      cv2.imshow("eyecameras", framequeue1[i])

del camera
cv2.destroyWindow("eyecameras")