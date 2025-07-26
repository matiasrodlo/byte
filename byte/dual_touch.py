#!/usr/bin/env python3
"""
Dual Touch Sensor Module for RobotDog

This module provides dual touch sensor functionality with slide detection.
It can detect individual touches on two sensors and sliding gestures between them.

Features:
- Individual touch detection on two sensors
- Slide gesture detection with configurable timing
- Debounced touch reading

Author: Robot Development Team
"""

from robot_hat import Pin
import time


class DualTouch():
    """
    Dual touch sensor controller with slide detection.
    
    This class manages two touch sensors and can detect individual touches
    as well as sliding gestures between the sensors.
    """

    SLIDE_MAX_INTERVAL = 0.5  # Maximum time interval for slide detection (seconds)

    def __init__(self, sw1='D2', sw2='D3'):
        """
        Initialize dual touch sensors.
        
        Args:
            sw1: GPIO pin for left touch sensor (default: 'D2')
            sw2: GPIO pin for right touch sensor (default: 'D3')
        """
        self.touch_L = Pin(sw1, mode=Pin.IN, pull=Pin.PULL_UP)  # Left touch sensor
        self.touch_R = Pin(sw2, mode=Pin.IN, pull=Pin.PULL_UP)  # Right touch sensor
        self.last_touch = 'N'      # Last detected touch ('L', 'R', 'LS', 'RS', 'N')
        self.last_touch_time = 0   # Timestamp of last touch detection

    # def read(self):
    #     if self.touch_L.value() == 1:
    #         time.sleep(0.1)
    #         if self.touch_R.value() == 1:
    #             return 'LS'
    #         else:
    #             return 'L'
    #     elif self.touch_R.value() == 1:
    #         time.sleep(0.1)
    #         if self.touch_L.value() == 1:
    #             return 'RS'
    #         else:
    #             return 'R'
    #     return 'N'

    def read(self):
        if self.touch_L.value() == 1:
            if self.last_touch == 'R' and\
                time.time() - self.last_touch_time <= self.SLIDE_MAX_INTERVAL:
                val = 'RS'
            else:
                val = 'L'
            self.last_touch_time = time.time()
            self.last_touch = 'L'
            return val
        elif self.touch_R.value() == 1:
            if self.last_touch == 'L' and\
                time.time() - self.last_touch_time <= self.SLIDE_MAX_INTERVAL:
                val = 'LS'
            else:
                val = 'R'
            self.last_touch_time = time.time()
            self.last_touch = 'R'
            return val
        return 'N'