#!/usr/bin/env python3
"""
Sound Effects Example
====================
Basic example showing how to play sound effects using the robot's speaker.

Note that you need to run with "sudo"

API:
    RobotDog.speak(name, volume=100)
        play sound effect in the file "../sounds"
        - name    str, file name of sound effect, no suffix required, eg: "angry"
        - volume  int, volume 0-100, default 100
"""
from byte import RobotDog
import os
import time

# change working directory
abspath = os.path.abspath(os.path.dirname(__file__))
# print(abspath)
os.chdir(abspath)

my_dog = RobotDog()

print("\033[033mNote that you need to run with \"sudo\", otherwise there may be no sound.\033[m")

# my_dog.speak("angry")
# time.sleep(2)

for name in os.listdir('../sounds'):
    name = name.split('.')[0] # remove suffix
    print(name)
    my_dog.speak(name)
    # my_dog.speak(name, volume=50)
    time.sleep(3) # Note that the duration of each sound effect is different
print("closing ...")
my_dog.close()