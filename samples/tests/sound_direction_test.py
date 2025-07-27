"""
Sound Direction Test Script
==========================
Tests the sound direction detection functionality.
"""

from byte.sound_direction import SoundDirection
from time import sleep


sd = SoundDirection()
while True:
    if sd.isdetected():
        print(f"Sound detected at {sd.read()} degrees")
    sleep(0.2)
