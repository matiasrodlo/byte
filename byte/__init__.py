#!/usr/bin/env python3
"""
RobotDog Package

This package provides the main RobotDog class and related utilities for controlling
a quadruped robot with various sensors and movement capabilities.

Main components:
- RobotDog: Main robot control class
- ActionDict: Predefined movement patterns and behaviors
- Various sensor and movement modules

Author: Robot Development Team
"""

from .byte import RobotDog
from robot_hat import utils
from time import sleep
from .version import __version__

def __main__():
    print(f"Thanks for using Byte {__version__} ! woof, woof, woof !")
    utils.reset_mcu()
    sleep(0.2)
