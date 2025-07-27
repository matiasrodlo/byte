"""
Stand Test Script
================
Tests the robot's standing action functionality.
"""

from byte import RobotDog
from time import sleep

my_dog = RobotDog()

SPEED = 95

my_dog.do_action("stand", speed=SPEED)

sleep(3)
my_dog.close()
