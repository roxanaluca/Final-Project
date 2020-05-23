import cv2
import os
import time
#import RPi.GPIO as gp 
import numpy as np
import time
import digitalio
import board

class Camera():
    
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
      time.sleep(1)
          
    def capture(self, cam):
        cmd = "raspistill -o picture_%d.jpg"%cam
        os.system(cmd)
        
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

    def __init__(self):
      super(Camera, self).__init__()
      self.pin7 = digitalio.DigitalInOut(board.D4)
      self.pin7.direction = digitalio.Direction.OUTPUT
      self.pin11 = digitalio.DigitalInOut(board.D17)
      self.pin11.direction = digitalio.Direction.OUTPUT
      self.pin12 = digitalio.DigitalInOut(board.D18)
      self.pin12.direction = digitalio.Direction.OUTPUT
      self.setup(0)
      self.setup(2)
      self.cameraid = 2