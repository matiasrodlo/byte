#!/usr/bin/env python3
"""
Dual Touch Reading Example
=========================
Basic example showing how to read the dual touch module status.

API:
    RobotDog.dual_touch.read()
        - return str, dual_touch status:
            - 'N'  no touch
            - 'L'  left touched
            - 'LS' left slide
            - 'R'  right touched
            - 'RS' right slide
"""

from byte import RobotDog
import time

my_dog = RobotDog()
while True:
    touch_status = my_dog.dual_touch.read()
    print(f"touch_status: {touch_status}")
    time.sleep(0.5)