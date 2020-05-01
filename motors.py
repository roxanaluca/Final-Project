#from adafruit_servokit import ServoKit
import time
import board
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_servokit import ServoKit

#print(board)
i2c = I2C(3)
#pca = adafruit_pca9685.PCA9685(i2c)
kit = ServoKit(channels=16, i2c=i2c)

"""kit.continuous_servo[0].throttle = 0.5
time.sleep(0.125)
#time.sleep(1)
kit.continuous_servo[0].throttle = 0
"""
#kit.servo[2].angle = 45