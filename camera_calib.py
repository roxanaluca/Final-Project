#!/usr/bin/python

#to access cv write commands
#source ~/.profile | workon cv 

import cv2
import os
import time
#import RPi.GPIO as gp 
from threading import Thread
import numpy as np
from PyQt5 import QtCore
import time
import digitalio
import board

class Camera(QtCore.QThread):
    
   grabbed_signal = QtCore.pyqtSignal([np.ndarray])
    
   def setup(self,id):
      if id == 0:
          i2c = "i2cset -y 1 0x70 0x00 0x04"
          os.system(i2c)
          self.pin7.value = False
          self.pin11.value = False
          self.pin12.value = True
          """gp.output(7, False)
          gp.output(11, False)
          gp.output(12, True)"""
      elif id == 1:
          i2c = "i2cset -y 1 0x70 0x00 0x05"
          os.system(i2c)
          self.pin7.value = True
          self.pin11.value = False
          self.pin12.value = True
          """gp.output(7, True)
          gp.output(11, False)
          gp.output(12, True)"""
      elif id == 2:
          i2c = "i2cset -y 1 0x70 0x00 0x06"
          os.system(i2c)
          self.pin7.value = False
          self.pin11.value = True
          self.pin12.value = False
          """gp.output(7, False)
          gp.output(11, True)
          gp.output(12, False)"""
      elif id == 3:
          i2c = "i2cset -y 1 0x70 0x00 0x07"
          os.system(i2c)
          self.pin7.value = True
          self.pin11.value = True
          self.pin12.value = False
          """gp.output(7, True)
          gp.output(11, True)
          gp.output(12, False)"""
      rval, frame = self.vc.read()
      time.sleep(1)
          
   def changeCamera(self,id):
      if self.cameraid == id:
          return
      if id == 0:
          self.pin7.value = False
          self.pin11.value = False
          self.pin12.value = True
          """gp.output(7, False)
          gp.output(11, False)
          gp.output(12, True)"""
      elif id == 1:
          self.pin7.value = True
          self.pin11.value = False
          self.pin12.value = True
          """gp.output(7, True)
          gp.output(11, False)
          gp.output(12, True)"""
      elif id == 2:
          self.pin7.value = False
          self.pin11.value = True
          self.pin12.value = False
          """gp.output(7, False)
          gp.output(11, True)
          gp.output(12, False)"""
      elif id == 3:
          self.pin7.value = True
          self.pin11.value = True
          self.pin12.value = False
          """gp.output(7, True)
          gp.output(11, True)
          gp.output(12, False)"""
      self.cameraid = id

   def __init__(self,cameraId):
      super(Camera, self).__init__()
      self.vc = cv2.VideoCapture(0)
      self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT,384)
      self.vc.set(cv2.CAP_PROP_FRAME_WIDTH,384)
      self.vc.set(cv2.CAP_PROP_FPS, 30)
      self.pin7 = digitalio.DigitalInOut(board.D4)
      self.pin7.direction = digitalio.Direction.OUTPUT
      self.pin11 = digitalio.DigitalInOut(board.D17)
      self.pin11.direction = digitalio.Direction.OUTPUT
      self.pin12 = digitalio.DigitalInOut(board.D18)
      self.pin12.direction = digitalio.Direction.OUTPUT
      self.setup(cameraId)
      self.runFlag = True
      
      
   def run(self):
    while self.runFlag and self.vc.isOpened():
      rval, frame = self.vc.read()
      if rval== False:
          break
      
      self.grabbed_signal.emit(frame)
      
    self.vc.release()
    
   def stop(self):
      self.runFlag = False
