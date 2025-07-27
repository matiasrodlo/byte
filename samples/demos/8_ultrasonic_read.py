#!/usr/bin/env python3
"""
Ultrasonic Distance Reading Example
==================================
Basic example showing how to read distance data from the ultrasonic sensor.

API:
    RobotDog.read_distance()
        return the distance read by ultrasound
        - return float

"""

from byte import RobotDog
import time

my_dog = RobotDog()
while True:
    distance = my_dog.read_distance()
    distance = round(distance,2)
    print(f"Distance: {distance} cm")
    time.sleep(0.5)
