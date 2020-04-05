import RPi.GPIO as gp
import os
import time
import cv2

gp.setwarnings(False)
gp.setmode(gp.BOARD)
gp.setup(7,gp.OUT)
gp.setup(11,gp.OUT)
gp.setup(12,gp.OUT)

i2c = "i2cset -y 1 0x70 0x00 0x04"
os.system(i2c)
gp.output(7, False)
gp.output(11, False)
gp.output(12, True)

vc = cv2.VideoCapture(0)
vc.set(cv2.CAP_PROP_FRAME_HEIGHT,300)
vc.set(cv2.CAP_PROP_FRAME_WIDTH,300)
vc.set(cv2.CAP_PROP_FPS, 64)

cv2.namedWindow("eyecameras")

while vc.isOpened():
    key = cv2.waitKey(40)
    if key == 27: # exit on ESC
      break
    rval, frame = vc.read()
    time.sleep(0.1)
    cv2.imshow("eyecameras", frame)
    
cv2.destroyWindow("eyecameras")
vc.release()