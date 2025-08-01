#!/usr/bin/env python3
"""
Preset Actions Example
=====================
Basic example showing how to use preset actions for common robot behaviors.

API:
    RobotDog.do_action(action_name, step_count=1, speed=50):
        do preset actions

        - action_name   str, name of preset actions, eg: "stand", "sit", "forward",
                        more to see: ../byte/actions_dictionary.py
        - step_count    int, times to perform the action
        - speed         int, speed of action, 0 ~ 100

more to see: ../byte/actions_dictionary.py

"""

from byte import RobotDog
import time

my_dog = RobotDog()

try:
    my_dog.do_action("stand", speed=60)
    my_dog.wait_all_done()

    my_dog.do_action("push_up", step_count=10, speed=60)
    my_dog.wait_all_done()

    my_dog.do_action("half_sit", speed=60)
    my_dog.wait_all_done()

    my_dog.do_action("wag_tail", step_count=80,speed=90)
    my_dog.do_action("tilting_head", step_count=5, speed=20)
    my_dog.wait_head_done()
    my_dog.body_stop()

except KeyboardInterrupt:
    pass
except Exception as e:
    print(f"\033[31mERROR: {e}\033[m")
finally:
    print("closing ...")
    my_dog.close()
