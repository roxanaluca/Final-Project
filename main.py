#!/usr/lib/python3

import cv2
from camera import Camera
import numpy as np
import RPi.GPIO as gp
import copy
import time
import math
import sys
import aruco
import os

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

class CameraParameters():
    
    def __init__(self):
        self.camm = aruco.CameraParameters()
        self.mtx = None
        self.dist = None
        self.h = None
        self.w = None
        self.cameratx = None
        self.mapx = None
        self.mapy = None

    def computeOptimalMatrix(self):
        self.cameratx, _ = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(self.h,self.w),0,(self.h,self.w))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None, self.cameratx,(self.h,self.w),5)
    
    def getFromFile(self,file_path):
        self.camm.readFromXMLFile(file_path)
        cv_file = cv2.FileStorage(file_path,cv2.FILE_STORAGE_READ)
        self.h = int(cv_file.getNode("image_width").real())
        self.w = int(cv_file.getNode("image_height").real())
        self.mtx = np.array(cv_file.getNode("camera_matrix").mat())
        self.dist = np.array(cv_file.getNode("distortion_coefficients").mat())

class Calibration():
    
    def __init__(self):
        self.imgpoints = []
        self.objpoints = []
        
        self.camera =[None,None]
        self.camera[0] = CameraParameters()
        self.camera[1] = CameraParameters()
        
        self.camera[0].getFromFile(os.path.join(os.path.dirname(__file__),"camera0.yaml"))
        self.camera[1].getFromFile(os.path.join(os.path.dirname(__file__),"camera2.yaml"))
        
        self.camera[0].computeOptimalMatrix()
        self.camera[1].computeOptimalMatrix()
        
    def undistort(self,frame, index):
        return cv2.remap(frame,self.camera[index].mapx,self.camera[index].mapy,cv2.INTER_LINEAR)
                
    def undistortPoints(self, points,index):
        rtemp = ttemp = np.array([0,0,0],dtype = 'float32')
        pointsout = cv2.undistortPoints(points,self.camera[index].cameratx,None)
        pointstemp = cv2.convertPointsToHomogeneous(pointsout)
        points1, _ = cv2.projectPoints(pointstemp,rtemp,ttemp,self.camera[index].mtx,self.camera[index].dist,pointsout)
        return points1
    
    def focal(self,index):
        return self.camera[index].cameratx[0][0]
        
    def cameraMatrixTI(self,index):
        return np.linalg.inv(self.camera[index].cameratx.T).dot(np.linalg.inv(self.camera[index].cameratx))
           
    def cameraParameters(self,index):
        return self.camera[index].camm
    
    def cameraMatrixI(self,index):
        return np.linalg.inv(self.camera[index].cameratx)
    
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
        self.angles = [None,None]
        self.detector = aruco.MarkerDetector()
        
    
    def startEndTracking(self):
        trackingValue = self.motorDriver.getIsTracking()
        if trackingValue:
            self.ui.tracking.setText("Start Tracking")
            self.motorDriver.setIsTracking(False)
        else:
            self.ui.tracking.setText("Stop Tracking")
            self.motorDriver.setIsTracking(True)
    
    def angle_compute_version_1(self,x,y,index):
        centerpoint = self.calibration.undistortPoints(np.array([[192,192]],dtype="float32"),index)
        angle = math.atan2(centerpoint[0][0][0] - x, 2*self.calibration.focal(index)) * 180 / math.pi
        return angle
        
    def angle_compute_version_2(self,x,y,index):
        mm = self.calibration.cameraMatrixTI(index)
        x1 = np.array([x,192,1])
        x2 = np.array([192,192,1])
        
        d1 = x1.T.dot(mm.dot(x2))
        d2 = x1.T.dot(mm.dot(x1))
        d3 = x2.T.dot(mm.dot(x2))
        
        angle = math.acos(d1/math.sqrt(d2*d3))*90/math.pi
        if x > 192:
            angle = angle*(-1)
        return angle
    
    def angle_compute_version_3(self,x,y,index):
        KImat = self.calibration.cameraMatrixI(index)
        worldcoord = KImat.dot(np.array([x,y,1],dtype="float32").T)
        print(worldcoord)
        XZ = worldcoord[0]/worldcoord[2]
        angle = math.acos(1.0/math.sqrt(1.0+XZ*XZ)) * 90 / math.pi
        if XZ > 0:
            angle = angle*(-1)
        return angle
    
    def angle_compute(self,points,index):
        (x,y) = [int(val) for val in self.initBB[index]]
        angle1 = self.angle_compute_version_1(x,y,index)
        angle2 = self.angle_compute_version_2(x,y,index)
        angle3 = self.angle_compute_version_3(x,y,index)
       
        print(str(angle1)+" "+str(angle2)+" "+str(angle3))
        self.motorDriver.moveMotor(2-index,angle2)
        self.angles[index] = angle2
    
    def update_photo(self, index, frame):
        frame_undistorted = self.calibration.undistort(frame,index)
        markers = self.detector.detect(frame, self.calibration.cameraParameters(index), False)
        if self.motorDriver.getIsTracking():
            if len(markers) == 0: 
                print("There is no marker")
            elif len(markers) > 1:
                print("There are many markers")
        for marker in markers:
            marker.draw(frame_undistorted,np.array([255,255,255]),2)
        if len(markers)==1:
            self.initBB[index] = markers[0].getCenter()
            x,y = self.initBB[index]
            cv2.circle(frame_undistorted,(x,y),1,(255,0,0),2)
        else:
            self.initBB[index] = None
        
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
