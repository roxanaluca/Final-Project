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

class Calibration():
    
    def __init__(self):
        self.imgpoints = []
        self.objpoints = []
        self.isCalibrate = False
    
    def calibrate(self,frame):
        
        criteria = (cv2.TERM_CRITERIA_EPS+ cv2.TERM_CRITERIA_MAX_ITER,5,0.001)
        
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,6), None)             
        print(ret)
        if ret == False:
            return
        
        imgp = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
        objp = np.zeros((6*7,3),np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
        self.imgpoints.append(imgp)
        self.objpoints.append(objp)
        ret,mtx,dist,rvecs,tvecs = cv2.calibrateCamera(self.objpoints,self.imgpoints,gray.shape[::-1],None,None)
        print(mtx)
        
        h,w = frame.shape[:2]
        self.cameratx, self.roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        print(self.cameratx)
        self.mtx = mtx
        self.dist = dist
        self.isCalibrate = True

    def undistort(self,frame):
        if self.isCalibrate:
            frame1 = cv2.undistort(frame,self.mtx,self.dist,None,self.cameratx)
            #x,y,w,h = self.roi
            #frame = frame1[y:y+h,x:x+w]
        return frame
    
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
        self.count = self.ui.count
        
        self.camera = Camera()
        self.camera.grabbed_signal.connect(self.update_photo)
        self.camera.start()
        
        self.ui.calibrate.clicked.connect(self.startCount)
        self.timer = 0
        self.iter = 2
        self.calibration = Calibration()
    
    def startCount(self):
        self.timer = 5
        self.iter = 0
    
    def update_photo(self, index, frame):
        
        frame_copy = self.detectSquare(self.calibration.undistort(frame))
        f_rgb = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)
        w,h = f_rgb.shape[:2]
        bytes_per_line = 3 * w
        qimg = QtGui.QImage(f_rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.ui_label_img_list[index].setPixmap(QtGui.QPixmap.fromImage(qimg))
        
        if self.timer > 0:
            if self.iter == 0:
                self.count.setText(self.timer)
            self.iter = self.iter + 1
            if self.iter%100 == 0:
                self.iter = 0
                self.timer = self.timer - 1
        if self.timer == 0 and self.iter < 2:
            self.count.setText("")
            print(index)
            self.calibration.calibrate(frame)
            self.iter = self.iter + 1
        
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
            cv2.polylines(frame, rectP, True, (0,255,0), thickness = 2,lineType = 8, shift = 0)
        return frame
    

if __name__=="__main__":
    gp.setwarnings(False)
    gp.setmode(gp.BOARD)
    gp.setup(7,gp.OUT)
    gp.setup(11,gp.OUT)
    gp.setup(12,gp.OUT)
    
    app = QtWidgets.QApplication(sys.argv)
    gui = CamGui()
    gui.show()
    sys.exit(app.exec_())
