#!/usr/bin/python

#to access cv write commands
#source ~/.profile | workon cv 

import cv2
import os
import time
import RPi.GPIO as gp 
from threading import Thread
import numpy as np
from PyQt5 import QtCore

class Camera(QtCore.QThread):
    
   grabbed_signal = QtCore.pyqtSignal([int,np.ndarray])
    
   def setup(self,id):
      if id == 0:
          i2c = "i2cset -y 1 0x70 0x00 0x04"
          os.system(i2c)
          gp.output(7, False)
          gp.output(11, False)
          gp.output(12, True)
      elif id == 1:
          i2c = "i2cset -y 1 0x70 0x00 0x05"
          os.system(i2c)
          gp.output(7, True)
          gp.output(11, False)
          gp.output(12, True)
      elif id == 2:
          i2c = "i2cset -y 1 0x70 0x00 0x06"
          os.system(i2c)
          gp.output(7, False)
          gp.output(11, True)
          gp.output(12, False)
      elif id == 3:
          i2c = "i2cset -y 1 0x70 0x00 0x07"
          os.system(i2c)
          gp.output(7, True)
          gp.output(11, True)
          gp.output(12, False)
      rval, frame = self.vc.read()
      time.sleep(1)
          
   def changeCamera(self,id):
      if self.cameraid == id:
          return
      if id == 0:
          gp.output(7, False)
          gp.output(11, False)
          gp.output(12, True)
      elif id == 1:
          gp.output(7, True)
          gp.output(11, False)
          gp.output(12, True)
      elif id == 2:
          gp.output(7, False)
          gp.output(11, True)
          gp.output(12, False)
      elif id == 3:
          gp.output(7, True)
          gp.output(11, True)
          gp.output(12, False)
      self.cameraid = id
      time.sleep(0.0001)

   def __init__(self):
      super(Camera, self).__init__()
      self.vc = cv2.VideoCapture(0)
      self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT,384)
      self.vc.set(cv2.CAP_PROP_FRAME_WIDTH,384)
      self.vc.set(cv2.CAP_PROP_FPS, 64)
      self.setup(0)
      self.setup(2)
      self.cameraid = 2
      self.runFlag = True
      
   def run(self):
    while self.runFlag and self.vc.isOpened():
      rval, frame = self.vc.read()
      if rval== False:
          break
      
      self.grabbed_signal.emit(0, frame)
      self.changeCamera(0)
      
      rval, frame = self.vc.read()
      if rval== False:
          break
      
      self.grabbed_signal.emit(1, frame)
      self.changeCamera(2)
      
    self.vc.release()
    
   def stop(self):
      self.runFlag = False
