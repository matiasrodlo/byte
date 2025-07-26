#!/usr/bin/env python3
"""
Action Dictionary for RobotDog

This module defines a dictionary of predefined robot actions and movements.
Each action is defined as a property that returns servo angles for different
robot parts (legs, head, tail).

Author: Robot Development Team
"""

from .byte import RobotDog
from .walk import Walk
from .trot import Trot
from math import sin

class ActionDict(dict):
    """
    Dictionary of predefined robot actions and movements.
    
    This class provides a collection of servo angle sequences for various
    robot behaviors including standing, walking, sitting, and expressive movements.
    Each action is implemented as a property that returns a tuple of (angles, part).
    """

    def __init__(self, *args, **kwargs):
        """Initialize the action dictionary with default parameters."""
        dict.__init__(self, *args, **kwargs)
        super().__init__()
        self.barycenter = -15  # Center of mass offset (mm)
        self.height = 95       # Default body height (mm)

    def __getitem__(self, item):
        """Get an action by name, converting spaces to underscores."""
        return eval("self.%s" % item.replace(" ", "_"))

    def set_height(self, height):
        """
        Set the robot's body height for standing position.
        
        Args:
            height: Height in mm (20-95 range)
        """
        if height in range(20, 95):
            self.height = height

    def set_barycenter(self, offset):
        """
        Set the center of mass offset for balance.
        
        Args:
            offset: Offset in mm (-60 to 60 range)
        """
        if offset in range(-60, 60):
            self.barycenter = offset

    @property
    def stand(self):
        """
        Standing position - robot stands upright on all four legs.
        
        Returns:
            tuple: (angles_list, 'legs') - Servo angles for standing position
        """
        x = self.barycenter  # Use configured center of mass offset
        y = 95               # Standing height
        return [
            RobotDog.legs_angle_calculation(
                [[x, y], [x, y], [x+20, y-5], [x+20, y-5]]),  # 4 leg positions
        ], 'legs'

    @property
    def sit(self):
        """
        Sitting position - robot sits with front legs extended and rear legs bent.
        
        Returns:
            tuple: (angles_list, 'legs') - Servo angles for sitting position
        """
        return [
            [30, 60, -30, -60, 80, -45, -80, 45],  # 8 servo angles for 4 legs
        ], 'legs'

    @property
    def lie(self):
        """
        Lying down position - robot lies flat on the ground.
        
        Returns:
            tuple: (angles_list, 'legs') - Servo angles for lying position
        """
        return [
            [45, -45, -45, 45, 45, -45, -45, 45]  # All legs extended flat
        ], 'legs'

    @property
    def lie_with_hands_out(self):
        """
        Lying position with front legs extended forward.
        
        Returns:
            tuple: (angles_list, 'legs') - Servo angles for extended lying position
        """
        return [
            [-60, 60, 60, -60, 45, -45, -45, 45],  # Front legs extended, rear legs normal
        ], 'legs'

    @property
    def forward(self):
        """
        Forward walking gait - robot moves forward in a straight line.
        
        Returns:
            tuple: (angles_list, 'legs') - Sequence of servo angles for forward walking
        """
        data = []
        forward = Walk(fb=Walk.FORWARD, lr=Walk.STRAIGHT)  # Create forward walking pattern
        coords = forward.get_coords()                       # Get coordinate sequence
        for coord in coords:
            data.append(RobotDog.legs_angle_calculation(coord))  # Convert to servo angles
        return data, 'legs'

    @property
    def backward(self):
        """
        Backward walking gait - robot moves backward in a straight line.
        
        Returns:
            tuple: (angles_list, 'legs') - Sequence of servo angles for backward walking
        """
        data = []
        backward = Walk(fb=Walk.BACKWARD, lr=Walk.STRAIGHT)  # Create backward walking pattern
        coords = backward.get_coords()                        # Get coordinate sequence
        for coord in coords:
            data.append(RobotDog.legs_angle_calculation(coord))  # Convert to servo angles
        return data, 'legs'

    @property
    def turn_left(self):
        """
        Left turn gait - robot turns to the left while moving forward.
        
        Returns:
            tuple: (angles_list, 'legs') - Sequence of servo angles for left turn
        """
        data = []
        turn_left = Walk(fb=Walk.FORWARD, lr=Walk.LEFT)  # Create left turn pattern
        coords = turn_left.get_coords()                   # Get coordinate sequence
        for coord in coords:
            data.append(RobotDog.legs_angle_calculation(coord))  # Convert to servo angles
        return data, 'legs'

    @property
    def turn_right(self):
        """
        Right turn gait - robot turns to the right while moving forward.
        
        Returns:
            tuple: (angles_list, 'legs') - Sequence of servo angles for right turn
        """
        data = []
        turn_right = Walk(fb=Walk.FORWARD, lr=Walk.RIGHT)  # Create right turn pattern
        coords = turn_right.get_coords()                    # Get coordinate sequence
        for coord in coords:
            data.append(RobotDog.legs_angle_calculation(coord))  # Convert to servo angles
        return data, 'legs'

    @property
    def trot(self):
        """
        Trotting gait - faster movement pattern using diagonal leg pairs.
        
        Returns:
            tuple: (angles_list, 'legs') - Sequence of servo angles for trotting
        """
        data = []
        trot = Trot(Trot.FORWARD, Trot.STRAIGHT)  # Create trotting pattern
        coords = trot.get_coords()                 # Get coordinate sequence
        for coord in coords:
            data.append(RobotDog.legs_angle_calculation(coord))  # Convert to servo angles
        return data, 'legs'

    @property
    def stretch(self):
        """
        Stretching pose - robot extends front and rear legs in opposite directions.
        
        Returns:
            tuple: (angles_list, 'legs') - Servo angles for stretching pose
        """
        return [
            [-80, 70, 80, -70, -20, 64, 20, -64],  # Front legs forward, rear legs back
        ], 'legs'

    @property
    def push_up(self):
        """
        Push-up exercise - robot performs a push-up motion.
        
        Returns:
            tuple: (angles_list, 'legs') - Two-step sequence for push-up motion
        """
        return [
            [90, -30, -90, 30, 80, 70, -80, -70],  # Down position
            [45, 35, -45, -35, 80, 70, -80, -70]   # Up position
        ], 'legs'

    # 打瞌睡 doze_off
    @property
    def doze_off(self):
        start = -30
        am = 20
        anl_f = 0
        anl_b = 0
        angs = []
        t = 4
        for i in range(0, am+1, 1):  # up
            anl_f = start + i
            anl_b = 45 - i
            angs += [[45, anl_f, -45, -anl_f, 45, -anl_b, -45, anl_b]]*t
            # print(1, anl_b)
        for _ in range(4):  # stop
            anl_f = start + am
            anl_b = 45 - am
            angs += [[45, anl_f, -45, -anl_f, 45, -anl_b, -45, anl_b]]*t
            # print(2, anl_b)
        for i in range(am, -1, -1):  # down
            anl_f = start + i
            anl_b = 45 - i
            angs += [[45, anl_f, -45, -anl_f, 45, -anl_b, -45, anl_b]]*t
            # print(3, anl_b)
        for _ in range(4):  # stop
            anl_f = start
            anl_b = 45
            angs += [[45, anl_f, -45, -anl_f, 45, -anl_b, -45, anl_b]]*t
            # print(4, anl_b)

        return angs, 'legs'

    # 点头昏睡 nod_lethargy
    @property
    def nod_lethargy(self):
        y = 0
        r = 0
        p = 30
        angs = []
        for i in range(21):
            r = round(10*sin(i*0.314), 2)
            p = round(10*sin(i*0.628) - 30, 2)
            if r == -10 or r == 10:
                for _ in range(10):
                    angs.append([y, r, p])
            angs.append([y, r, p])

        return angs, 'head'

    # 摇头 shake_head
    @property
    def shake_head(self):
        amplitude = 60
        angs = []
        for i in range(21):
            y = round(sin(i*0.314), 2)
            y1 = amplitude*sin(i*0.314)
            angs.append([y1, 0, 0])
        return angs, 'head'

    # 左歪头 tilting_head_left
    @property
    def tilting_head_left(self):
        yaw = 0
        roll = -25
        pitch = 15
        return [
            [yaw, roll, pitch]
        ], 'head'

    # 右歪头 tilting_head_right
    @property
    def tilting_head_right(self):
        yaw = 0
        roll = 25
        pitch = 20
        return [
            [yaw, roll, pitch]
        ], 'head'

    # 左右歪头 tilting_head left and right
    @property
    def tilting_head(self):
        yaw = 0
        roll = 22
        pitch = 20
        return [[yaw, roll, pitch]]*20 \
            + [[yaw, -roll, pitch]]*20, 'head'

    # 仰头吠叫 head_bark
    @property
    def head_bark(self):
        return [[0, 0, -40],
                [0, 0, -10],
                [0, 0, -10],
                [0, 0, -40],
                ], 'head'

    # 摇尾巴 wag_tail
    @property
    def wag_tail(self):
        # amplitude = 50
        # angs = []
        # for i in range(21):
        #     a = round(sin(i*0.314), 2)
        #     angs.append([amplitude*a])
        angs = [[-30], [30]]
        return angs, 'tail'

    # head_up_down
    @property
    def head_up_down(self):
        # amplitude = 20
        # angs = []
        # for i in range(20):
        #     y = round(sin(i*0.314),3)
        #     y1 = amplitude*sin(i*0.314)
        #     if y == -1 or y == 1:
        #         for _ in range(10):
        #             angs.append([0,0,y1])
        #     angs.append([0,0,y1])
        # return angs,'head'
        return [
            [0, 0, 20],
            [0, 0, 20],
            [0, 0, -10]
        ], 'head'

    # half_sit
    @property
    def half_sit(self):
        return [
            [25, 25, -25, -25, 64, -45, -64, 45],
        ], 'legs'
