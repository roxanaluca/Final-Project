from camera_pano import Camera
import numpy as np
import cv2
import time

from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_servokit import ServoKit

class MotorDriver():
    def __init__(self):
        i2c = I2C(3)
        self.kit = ServoKit(channels=16, i2c=i2c)
    
    def moveMotor(self,index, angle):
        if index <1 and index>2:
            return
        currentangle = self.kit.servo[index].angle
        if currentangle+angle < 1 or currentangle+angle > 179:
            return
        self.kit.servo[index].angle = currentangle + angle

def trainAndIdentify(index,image):
    desc = None
    if index == 0:
        desc = cv2.xfeatures2d.SIFT_create()
    elif index == 1:
        desc = cv2.xfeatures2d.SURF_create()
    elif index == 2:
        desc = cv2.BRISK_create()
    else:
        desc = cv2.ORB_create()
    
    return desc.detectAndCompute(image, None)

def createMatcherObject(index,crossCheck):
    if index < 2:
        return cv2.BFMatcher(cv2.NORM_L2,crossCheck = crossCheck)
    return cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck = crossCheck)

def findCorrectHomography(matches,keypoints1,keypoints2,ratio):
    good = []
    for i,m in enumerate(matches):
        if i < len(matches) -1 and m.distance < ratio*matches[i+1].distance:
            good.append(m)
    if len(good) < 10:
           return None
    src_pts = np.float32([keypoints1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
    dst_pts = np.float32([keypoints2[n.trainIdx].pt for n in good]).reshape(-1,1,2)
    M,mask = cv2.findHomography(src_pts,dst_pts, cv2.RANSAC,5.0)
    return M

def panorama_processing(index, frames):
    
    (keypoints, feature) = trainAndIdentify(index, frames[0])
    
    bf = createMatcherObject(index,True)
    result = None
    (keypoints1, feature1) = trainAndIdentify(index,frames[1])
    matches = bf.match(feature,feature1)
    H = findCorrectHomography(matches, keypoints,keypoints1,0.4)
    if H is None:
        return None
    width = frames[0].shape[1]+frames[1].shape[1]
    height = frames[0].shape[0]
    print(H)
    result = cv2.warpPerspective(frames[0],H,(width,height))
    result[0:frames[1].shape[0],0:frames[1].shape[1]] = frames[1]
    return result

def take_picture(camera,no):
    frame1 = camera.capture(no)
    time.sleep(1)
    return cv2.imread('picture_'+str(no)+".jpg")
    
if __name__=="__main__":
    camera = Camera()
    motorDriver = MotorDriver()
    frame1 = take_picture(camera,1)
    time.sleep(0.2)
    motorDriver.moveMotor(1,30)
    frame2 = take_picture(camera,2)
    
    pan = panorama_processing(3,[frame1,frame2])
    if pan is not None:
        cv2.imwrite("panorama.jpg",pan)
    