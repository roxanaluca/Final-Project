#!/usr/lib/python3

import cv2
from camera import Camera
import numpy as np
import RPi.GPIO as gp
import copy
import time
import math
import sys

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic
with open('gui_ui.py', 'w') as fd:
    uic.compileUi('gui/gui.ui', fd)
import gui_ui as gui_ui

from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_servokit import ServoKit

class MotorDriver():
    def __init__(self):
        i2c = I2C(3)
        self.kit = ServoKit(channels=16, i2c=i2c)
        self.isTracking = False
    
    def moveMotor(self,index, angle):
        if index <1 and index>2:
            return
        if self.isTracking == False:
            return
        currentangle = self.kit.servo[index].angle
        if currentangle+angle < 1 or currentangle+angle > 179:
            return
        self.kit.servo[index].angle = currentangle + angle
     
    def getIsTracking(self):
         return self.isTracking
    
    def setIsTracking(self,value):
         self.isTracking = value

class Calibration():
    
    def __init__(self):
        self.imgpoints = []
        self.objpoints = []
        
        h = 384
        w = 384
        
        matrix = np.loadtxt("camera0.txt",dtype=np.float)
        self.mtx = matrix[0:3,0:3]
        self.dist = matrix[3,:]
       
        self.cameratx, _ = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),0,(w,h))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None, self.cameratx,(w,h),5)
        
        matrix = np.loadtxt("camera2.txt",dtype=np.float)
        self.mtx1 = matrix[0:3,0:3]
        self.dist1 = matrix[3,:]
       
        self.cameratx1, _ = cv2.getOptimalNewCameraMatrix(self.mtx1,self.dist1,(w,h),0,(w,h))
        self.mapx1, self.mapy1 = cv2.initUndistortRectifyMap(self.mtx1,self.dist1,None, self.cameratx1,(w,h),5)

    def undistort(self,frame, index):
        if index == 0:
            return cv2.remap(frame,self.mapx,self.mapy,cv2.INTER_LINEAR)
        else:
            return cv2.remap(frame,self.mapx1,self.mapy1,cv2.INTER_LINEAR)
    
    def undistortPoints(self, points,index):
        points1 = points.astype(np.float32)
        if index == 0:
            points1 = cv2.undistortPoints(points1,self.mtx,self.dist,None,self.cameratx)
        else:
            points1 = cv2.undistortPoints(points1,self.mtx1,self.dist1,None,self.cameratx1)
        return points1
    
    def focal(self,index):
        if index == 0:
            return self.cameratx[0][0]
        else:
            return self.cameratx1[0][0]
        
class CamGui( QtWidgets.QMainWindow ):
    
    def __init__(self, *args):
        super(CamGui, self).__init__(*args)
        self.ui = gui_ui.Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.ui_label_img_list = [self.ui.label_img0, self.ui.label_img1]
        self.img_no = len(self.ui_label_img_list)
        
        for l in self.ui_label_img_list:
            l.mouseMoveEvent = self.on_mouse_move
        self.ui.label_img0.mouseReleaseEvent = self.on_mouse_release_label_img
        self.ui.label_img1.mouseReleaseEvent = self.on_mouse_release_label_img
        
        self.ui.tracking.clicked.connect(self.startEndTracking)
        self.motorDriver = MotorDriver()
        
        self.camera = Camera()
        self.camera.grabbed_signal.connect(self.update_photo)
        self.camera.start()
        
        self.calibration = Calibration()
        self.initBB = [None,None]
        self.tracker0 = cv2.TrackerMOSSE_create()
        self.tracker1 = cv2.TrackerMOSSE_create()
    
    def startEndTracking(self):
        trackingValue = self.motorDriver.getIsTracking()
        if trackingValue:
            self.ui.tracking.setText("Start Tracking")
            self.motorDriver.setIsTracking(False)
        else:
            self.ui.tracking.setText("Stop Tracking")
            self.motorDriver.setIsTracking(True)
    
    def angle_compute(self,points,index):
        (x,y,w,h) = [int(val) for val in self.initBB[index]]
        points = self.calibration.undistortPoints(np.array([[[x,y],[x+w,y+h]]]),index)    
        pointsCenter = self.calibration.undistortPoints(np.array([[[193,193]]]),index)
        centerPoint = (points[0]+points[1])*0.5
        rightAngle = math.atan2(pointsCenter[0][0][0] - centerPoint[0][0] , self.calibration.focal(index)) * 180 / math.pi
        print(str(pointsCenter[0][0][0] - centerPoint[0][0])+" " +str(rightAngle))
        self.motorDriver.moveMotor(2-index,rightAngle)
    
    def update_photo(self, index, frame):
        frame_undistorted = self.calibration.undistort(frame,index)
        
        success = False
        if self.initBB[index] is not None:
            if index == 0:
                (success, self.initBB[0]) = self.tracker0.update(frame_undistorted)
            else:
                (success, self.initBB[1]) = self.tracker1.update(frame_undistorted)
            (x,y,w,h) = [int(val) for val in self.initBB[index]] 
            self.initBB[index] = (x,y,w,h)
        
        if self.initBB[index] is None or success == False:
            self.initBB[index] = self.detectSquare(frame_undistorted)
            if self.initBB[index] is not None:
                if index == 0:
                    self.tracker0.init(frame_undistorted, self.initBB[0])
                else:
                    self.tracker1.init(frame_undistorted, self.initBB[1])
        
        if self.initBB[index] is not None:
            cv2.rectangle(frame_undistorted, self.initBB[index], (0,255,0), thickness = 2,lineType = 8, shift = 0)
        
        f_rgb = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2RGB)
        w,h = f_rgb.shape[:2]
        bytes_per_line = 3 * w
        qimg = QtGui.QImage(f_rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.ui_label_img_list[index].setPixmap(QtGui.QPixmap.fromImage(qimg))
       
        if self.initBB[index] is not None and self.motorDriver.getIsTracking():
            self.angle_compute(self.initBB[index],index)
        
    def on_mouse_release_label_img(self, ev):
        print('why you click me ?!')

    def on_mouse_move(self, e):
        pass
    
    def detectSquare(self,frame):    
    
        median = cv2.GaussianBlur(frame,(5,5),0)
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
        sensitivityWhite = 100
        sensitivityBlack = 100
        lower_white = np.array([0,0,255-sensitivityWhite], np.uint8)
        upper_white = np.array([255,sensitivityWhite,255], np.uint8)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        lower_black = np.array([0,0,0], np.uint8)
        upper_black = np.array([255,255,sensitivityBlack], np.uint8)
        mask_black = cv2.inRange(hsv, lower_black, upper_black)
        mask =  cv2.addWeighted(mask_white,0.3,cv2.bitwise_not(mask_black),0.7,0)
        _, im_bw = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)

        cont = []
        contnew = []
        cont,_ = cv2.findContours(image = im_bw, mode = cv2.RETR_LIST, method = cv2.CHAIN_APPROX_SIMPLE,contours = cont)
        accuracyRate = 0.03
        cols = 5
        rows = 5

        minPerimeterRate = 0.037
        maxPerimeterRate = 4

        minPerimeterPixels = minPerimeterRate * max(cols, rows) * 1000
        maxPerimeterPixels = maxPerimeterRate * max(cols, rows) * 1000
        minCornerDistanceRate = 0.05
        
        minAspectRatio = 1.0
        maxAspectRatio = 1.25

        for i in range(0,len(cont) - 1):
            rectP = cv2.approxPolyDP(cont[i], cv2.arcLength(cont[i],True) * accuracyRate, True)
            if len(rectP) != 4 or (not cv2.isContourConvex(rectP)):
                continue

            if minPerimeterPixels > cv2.arcLength(rectP,True) or maxPerimeterPixels < cv2.arcLength(rectP,True):
                 continue

            frame_rows,frame_cols = frame.shape[:2]
            minDistSq = math.pow(max(frame_rows,frame_cols),2)
            maxDistSq = 0
            for j in range(4):
                d = math.pow(rectP[j][0][0] - rectP[(j+1)%4][0][0],2) + math.pow(rectP[j][0][1] - rectP[(j+1)%4][0][1],2)
                minDistSq = min(minDistSq,d)
                maxDistSq = max(maxDistSq,d)
            minCornerDistancePixels = math.pow(cv2.arcLength(rectP,True)*minCornerDistanceRate,2)
            if minDistSq < minCornerDistancePixels or maxDistSq/minDistSq > maxAspectRatio or maxDistSq/minDistSq < minAspectRatio:
                continue
            contnew.append(cv2.boundingRect(rectP))
        if len(contnew) == 1:
            return contnew[0]
        elif len(contnew) == 0:
            if self.motorDriver.getIsTracking() == True:
                print("No object detected")
        else:
            if self.motorDriver.getIsTracking() == True: 
                print("Too many objects detected")
        return None
    

if __name__=="__main__":
    """gp.setwarnings(False)
    gp.setmode(gp.BOARD)
    gp.setup(7,gp.OUT)
    gp.setup(11,gp.OUT)
    gp.setup(12,gp.OUT)"""
    
    app = QtWidgets.QApplication(sys.argv)
    gui = CamGui()
    gui.show()
    sys.exit(app.exec_())
