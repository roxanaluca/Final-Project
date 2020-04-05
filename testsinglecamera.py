import time
import cv2

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