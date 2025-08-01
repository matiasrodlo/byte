"""
Tail Wagging Test Script
=======================
Tests the robot's tail wagging action functionality.
"""

from byte import RobotDog
from time import sleep

my_dog = RobotDog()

SPEED = 100

my_dog.do_action("wag_tail", step_count=40, speed=SPEED)
my_dog.wait_all_done()

my_dog.close()
