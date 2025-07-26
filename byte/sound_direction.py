#!/usr/bin/env python3
"""
Sound Direction Detection Module for RobotDog

This module provides sound direction recognition capabilities using a microphone array
and SPI communication with a TR16F064B chip. It can detect sound direction in a full
360-degree range with 20-degree resolution.

Communication Protocol:
- Master sends 16-bit data to slave
- Master receives 16-bit response from slave
- MSB-first transmission
- BUSY line controls detection cycle

Author: Robot Development Team
"""
'''
                Sound Direction Recognition Module – Communication Protocol

1. Communication Format:
   - The master (controller) randomly sends 16-bit data to the slave.
   - Then, it receives 16-bit data from the slave.

   Format of the 16-bit data received by the master:
     (1) First receive the lower 8 bits, then receive the higher 8 bits.
     (2) MSB-first (Most Significant Bit first) transmission is used.

2. Direction detection:
   - The module can detect a full 360-degree range.
   - The smallest resolution unit is 20 degrees, meaning the data range is 0 to 355.

3. Operation Process:
   - The master pulls the BUSY line high, triggering the TR16F064B chip to start monitoring sound direction.
   - When the chip detects a direction, it pulls the BUSY line low (normally it stays high).
   - The master, upon detecting that BUSY is low, sends a 16-bit arbitrary command to the chip.
   - It then receives a 16-bit response.
   - Once reception is complete, the master pulls the BUSY line high again to resume monitoring.
'''

import spidev
from gpiozero import OutputDevice, InputDevice


class SoundDirection():
    """
    Sound direction detection using microphone array and SPI communication.
    
    This class interfaces with a TR16F064B chip to detect sound direction
    in a 360-degree range with 20-degree resolution.
    """
    
    CS_DELAY_US = 500       # Chip select delay in microseconds
    CLOCK_SPEED = 10000000  # SPI clock speed (10 MHz)

    def __init__(self, busy_pin=6):
        """
        Initialize sound direction detection.
        
        Args:
            busy_pin: GPIO pin number for BUSY signal
        """
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Open SPI bus 0, device 0
        self.busy = InputDevice(busy_pin, pull_up=False)  # BUSY signal input

    def read(self):
        result = self.spi.xfer2([0, 0, 0, 0, 0, 0], self.CLOCK_SPEED,
                                self.CS_DELAY_US)


        l_val, h_val = result[4:]  # ignore the fist two values
        # print([h_val, l_val])
        if h_val == 255:
            return -1
        else:
            val = (h_val << 8) + l_val
            val = (360 + 160 - val) % 360  # Convert zero
            return val

    def isdetected(self):
        return self.busy.value == 0


if __name__ == '__main__':
    from time import sleep
    sd = SoundDirection()
    while True:
        if sd.isdetected():
            print(f"Sound detected at {sd.read()} degrees")
        sleep(0.2)