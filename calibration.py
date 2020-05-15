#!/usr/lib/python3

import cv2
from camera_calib import Camera
import numpy as np
import RPi.GPIO as gp
import copy
import time
import math
import sys
import os

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic
with open('gui_ui_calib.py', 'w') as fd:
    uic.compileUi('gui/gui_calib.ui', fd)
import gui_ui_calib as gui_ui

class Calibration():
    
    def __init__(self):
        self.imgpoints = []
        self.objpoints = []
        self.isCalibrate = False
    
    def calibrate(self,frame):
        
        criteria = (cv2.TERM_CRITERIA_EPS+ cv2.TERM_CRITERIA_MAX_ITER,5,0.001)
        
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)             
        print(ret)
        if ret == False:
            return
        
        imgp = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
        objp = np.zeros((6*9,3),np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
        self.imgpoints.append(imgp)
        self.objpoints.append(objp)
        ret,self.mtx,self.dist,rvecs,tvecs = cv2.calibrateCamera(self.objpoints,self.imgpoints,gray.shape[::-1],None,None)
        
        h,w = frame.shape[:2]
        cv_file = cv2.FileStorage(os.path.join(os.path.dirname(__file__),"camera"+str(INDEX)+".yaml"), cv2.FILE_STORAGE_WRITE)
        cv_file.write('image_width',h)
        cv_file.write('image_height',w)
        cv_file.write('camera_matrix',self.mtx)
        cv_file.write('distortion_coefficients',self.dist)
        cv_file.release()
        
        self.cameratx, _ = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),0,(w,h))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None, self.cameratx,(w,h),5)
        self.isCalibrate = True

    def undistort(self,frame):
        if self.isCalibrate:
            return cv2.remap(frame,self.mapx,self.mapy,cv2.INTER_LINEAR)
        return frame
    
class CamGui( QtWidgets.QMainWindow ):
    
    def __init__(self, *args):
        super(CamGui, self).__init__(*args)
        self.ui = gui_ui.Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.ui_label_img_list = [self.ui.label_img0]
        self.img_no = len(self.ui_label_img_list)
        
        for l in self.ui_label_img_list:
            l.mouseMoveEvent = self.on_mouse_move
        self.ui.label_img0.mouseReleaseEvent = self.on_mouse_release_label_img
        self.count = self.ui.count
        
        self.camera = Camera(INDEX)
        self.camera.grabbed_signal.connect(self.update_photo)
        self.camera.start()
        
        self.ui.calibrate.clicked.connect(self.startCount)
        self.timer = 0
        self.iter = 1
        self.calibration = Calibration()
    
    def startCount(self):
        self.timer = 5
        self.iter = 0
    
    def update_photo(self, frame):
        frame_copy = self.calibration.undistort(frame)
        f_rgb = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)
        w,h = f_rgb.shape[:2]
        bytes_per_line = 3 * w
        qimg = QtGui.QImage(f_rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        self.ui_label_img_list[0].setPixmap(QtGui.QPixmap.fromImage(qimg))
        
        if self.timer > 0:
            if self.iter == 0:
                self.count.setText(self.timer)
            self.iter = self.iter + 1
            if self.iter%50 == 0:
                self.iter = 0
                self.timer = self.timer - 1
        elif self.timer == 0 and self.iter < 1:
            self.count.setText("")
            self.calibration.calibrate(frame)
            self.iter = self.iter + 1
        
    def on_mouse_release_label_img(self, ev):
        print('why you click me ?!')

    def on_mouse_move(self, e):
        pass
    

if __name__=="__main__":
    
    INDEX = 0
    app = QtWidgets.QApplication(sys.argv)
    gui = CamGui()
    gui.show()
    sys.exit(app.exec_())

