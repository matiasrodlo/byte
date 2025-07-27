#!/usr/bin/env python3
"""
IMU Data Reading Example
=======================
Basic example showing how to read IMU (SH3001) data from the robot.

SH3001: IMU with integrated 3-axis accelerometer and 3-axis gyroscope.
Pay attention to the installation direction of the module when using.

The data "accData" and "gyroData" of the IMU will be continuously refreshed
in the built-in thread of the RobotDog class.

API:
    RobotDog.accData = [ax, ay, az]
        the acceleration value, default gravity 1G = -16384
        note that the accelerometer direction is opposite to the actual acceleration direction

    RobotDog.gyroData = [gx, gy, gz]
        the gyro value

more to see: ../byte/sh3001.py

"""

from byte import RobotDog
import time

my_dog = RobotDog()

while True:
    ax, ay, az = my_dog.accData
    gx, gy, gz = my_dog.gyroData
    print(f"accData: {ax}, {ay}, {az}       gyroData: {gx}, {gy}, {gz}")
    time.sleep(0.2)