#!/usr/bin/env python3
"""
Sound Direction Reading Example
=============================
Basic example showing how to read the azimuth of sound direction using the robot's sound direction module.

API:
    RobotDog.ears.isdetected():
        return    bool, whether the sound direction recognition module has detected the sound

    RobotDog.ears.read()
        return    int, the azimuth of the identified sound, 0 ~ 359

more to see: ../byte/sound_direction.py

"""

from byte import RobotDog

my_dog = RobotDog()

while True:
    if my_dog.ears.isdetected():
        direction = my_dog.ears.read()
        print(f"sound direction: {direction}")
