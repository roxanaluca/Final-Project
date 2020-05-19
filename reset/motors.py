#from adafruit_servokit import ServoKit
import time
import board
import busio
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_servokit import ServoKit

#print(board)
i2c = I2C(3)
kit = ServoKit(channels=16, i2c=i2c)

"""kit.continuous_servo[0].throttle = 0.5
time.sleep(0.125)
#time.sleep(1)
kit.continuous_servo[0].throttle = 0"""

kit.servo[1].angle = 30
kit.servo[2].angle = 75
#time.sleep(0.2)
#kit.servo[1].angle = 15
#print(kit.servo[1].angle)
#kit.servo[1].angle = 30