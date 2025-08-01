#!/usr/bin/env python3
"""
RobotDog Rest Example
===================
Advanced example demonstrating rest and sleep behavior with sound detection.
"""

from byte import RobotDog
from time import sleep
from preset_actions import shake_head

my_dog = RobotDog()
sleep(0.1)

def loop_around(amplitude=60, interval=0.5, speed=100):
    my_dog.head_move([[amplitude,0,0]], immediately=True, speed=speed)
    my_dog.wait_all_done()
    sleep(interval)
    my_dog.head_move([[-amplitude,0,0]], immediately=True, speed=speed)
    my_dog.wait_all_done()
    sleep(interval)
    my_dog.head_move([[0,0,0]], immediately=True, speed=speed)
    my_dog.wait_all_done()

def is_sound():
    if my_dog.ears.isdetected():
        direction = my_dog.ears.read()
        if direction != 0:
            return True
        else:
            return False
    else:
        return False

def rest():
    my_dog.wait_all_done()
    my_dog.do_action('lie', speed=50)
    my_dog.wait_all_done()

    while True:
        # Sleeping
        my_dog.rgb_strip.set_mode('breath', 'pink', bps=0.3)
        my_dog.head_move([[0,0,-40]], immediately=True, speed=5)
        my_dog.do_action('doze_off', speed=92)
        # Cleanup sound detection
        sleep(1)
        is_sound()

        # keep sleeping
        while is_sound() is False:
            my_dog.do_action('doze_off', speed=92)
            sleep(0.2)

        # If heard anything, wake up
        # Set light to yellow and stand up
        my_dog.rgb_strip.set_mode('boom', 'yellow', bps=1)
        my_dog.body_stop()
        # Add a delay for the body to stopb
        sleep(0.1)
        my_dog.do_action('stand', speed=80)
        my_dog.head_move([[0, 0, 0]], immediately=True, speed=80)
        my_dog.wait_all_done()
        # Look arround
        loop_around(60, 1, 60)
        sleep(0.5)
        # tilt head and being confused
        my_dog.speak('confused_3', volume=80)
        my_dog.do_action('tilting_head_left', speed=80)
        my_dog.wait_all_done()
        sleep(1.2)
        my_dog.head_move([[0, 0, -10]], immediately=True, speed=80)
        my_dog.wait_all_done()
        sleep(0.4)
        # Shake head , mean to ignore it
        shake_head(my_dog)
        sleep(0.2)

        # Lay down again
        my_dog.rgb_strip.set_mode('breath', 'pink', bps=1)
        my_dog.do_action('lie', speed=50)
        my_dog.wait_all_done()
        sleep(1)


if __name__ == "__main__":
    try:
        rest()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        my_dog.close()
