
"""
Trotting Gait Implementation for RobotDog

This module implements a trotting gait pattern for quadruped robot movement.
A trotting gait uses diagonal leg pairs moving in synchrony for faster movement.

Gait Pattern:
leg| 0 | 1 |
====|===|===|
 1  |^^^|___|  (Front-left leg up)
 2  |___|^^^|  (Rear-right leg up)  
 3  |___|^^^|  (Front-right leg up)
 4  |^^^|___|  (Rear-left leg up)

Movement sequence: 1, 4, 2, 3 (diagonal pairs)
Each section is divided into multiple steps for smooth motion.

Author: Robot Development Team
"""

#!/usr/bin/env python3

import readchar
from time import sleep as delay
from math import cos, pi


class Trot():
    """
    Trotting gait generator for quadruped robot movement.
    
    This class generates coordinate sequences for trotting movement patterns,
    where diagonal leg pairs move in synchrony for efficient locomotion.
    """

    # Movement direction constants
    FORWARD = 1
    BACKWARD = -1
    LEFT = -1
    STRAIGHT = 0
    RIGHT = 1

    # Gait configuration
    SECTION_COUNT = 2        # Number of sections in a complete gait cycle
    STEP_COUNT = 3           # Number of steps per section
    LEG_RAISE_ORDER = [[1, 4], [2, 3]]  # Diagonal leg pairs: [front-left, rear-right], [front-right, rear-left]
    
    # Physical parameters (in mm)
    # ==========================
    # Gait parameters optimized for trotting locomotion with diagonal leg pairs.
    # Values determined through empirical testing and stability analysis.
    # 
    # Trotting gait characteristics:
    # - Duty factor: 0.5 (50% contact time, 50% swing phase)
    # - Diagonal pairs: Front-left + Rear-right, Front-right + Rear-left
    # - Stability: 2-point support during swing phase
    LEG_STEP_HEIGHT = 20     # Height of leg lift during stepping (mm) - ground clearance
    LEG_STEP_WIDTH = 100     # Width of leg movement (mm) - forward stride length
    CENTER_OF_GRAVITY = -17  # Body center of gravity offset (mm) - balance point adjustment
    LEG_STAND_OFFSET = 5     # Leg standing position offset (mm) - neutral stance adjustment
    Z_ORIGIN = 80            # Base height of robot body (mm) - default standing height

    TURNING_RATE = 0.5
    LEG_STAND_OFFSET_DIRS = [-1, -1, 1, 1]
    LEG_STEP_SCALES_LEFT = [TURNING_RATE, 1, TURNING_RATE, 1]
    LEG_STEP_SCALES_MIDDLE = [1, 1, 1, 1]
    LEG_STEP_SCALES_RIGHT = [1, TURNING_RATE, 1, TURNING_RATE]
    LEG_ORIGINAL_Y_TABLE = [0, 1, 1, 0]
    LEG_STEP_SCALES = [LEG_STEP_SCALES_LEFT,
                       LEG_STEP_SCALES_MIDDLE, LEG_STEP_SCALES_RIGHT]

    def __init__(self, fb, lr):
        """
            Trot init
            fb: FORWARD(1) or BACKWARD(-1)
            lr: LEFT(1), STRAIGHT(0) or RIGHT(-1)
        """
        self.fb = fb
        self.lr = lr

        if self.fb == self.FORWARD:
            if self.lr == self.STRAIGHT:
                self.y_offset = 0 + self.CENTER_OF_GRAVITY
            else:
                self.y_offset = -2 + self.CENTER_OF_GRAVITY
        elif self.fb == self.BACKWARD:
            if self.lr == self.STRAIGHT:
                self.y_offset = 8 + self.CENTER_OF_GRAVITY
            else:
                self.y_offset = 1 + self.CENTER_OF_GRAVITY
        else:
            self.y_offset = self.CENTER_OF_GRAVITY
        self.leg_step_width = [
            self.LEG_STEP_WIDTH * self.LEG_STEP_SCALES[self.lr+1][i] for i in range(4)]
        self.section_length = [self.leg_step_width[i] /
                               (self.SECTION_COUNT-1) for i in range(4)]
        self.step_down_length = [
            self.section_length[i] / self.STEP_COUNT for i in range(4)]
        self.leg_offset = [self.LEG_STAND_OFFSET *
                           self.LEG_STAND_OFFSET_DIRS[i] for i in range(4)]
        self.leg_origin = [self.leg_step_width[i] / 2 + self.y_offset + (
            self.leg_offset[i] * self.LEG_STEP_SCALES[self.lr+1][i]) for i in range(4)]

    def step_y_func(self, leg, step):
        """
        Trajectory function for forward/backward leg motion (Y-axis).
        
        Uses a cosine trajectory to generate smooth leg motion during the swing phase.
        The trajectory follows a half-cosine wave from origin to target position.
        
        Mathematical Formulation:
        ------------------------
        θ = step · π / (STEP_COUNT - 1)  # Normalized phase [0, π]
        y(step) = y_origin + (step_width/2) · (cos(θ) - direction) · direction
        
        This creates a smooth S-curve motion that minimizes acceleration at trajectory
        endpoints, reducing mechanical stress and improving stability.
        
        Parameters:
        -----------
        leg : int
            Leg index (0-3): Front-left, Front-right, Rear-left, Rear-right
        step : int
            Current step index within the section (0 to STEP_COUNT-1)
        
        Returns:
        --------
        float
            Y-coordinate (forward position) in millimeters
        
        Notes:
        ------
        The cosine trajectory provides:
        - Zero velocity at endpoints (smooth start/stop)
        - Maximum velocity at midpoint
        - Continuous acceleration profile
        """
        theta = step * pi / (self.STEP_COUNT-1)
        temp = (self.leg_step_width[leg] *
                (cos(theta) - self.fb) / 2 * self.fb)
        y = self.leg_origin[leg] + temp
        return y

    def step_z_func(self, step):
        """
        Trajectory function for vertical leg motion (Z-axis).
        
        Uses a linear trajectory for vertical leg lift during swing phase.
        The leg lifts from ground level (Z_ORIGIN) to maximum height (Z_ORIGIN - STEP_HEIGHT).
        
        Mathematical Formulation:
        ------------------------
        z(step) = Z_ORIGIN - STEP_HEIGHT · (step / (STEP_COUNT - 1))
        
        This creates a constant-velocity vertical motion, which is acceptable for
        the relatively small vertical displacement compared to horizontal motion.
        
        Parameters:
        -----------
        step : int
            Current step index within the section (0 to STEP_COUNT-1)
        
        Returns:
        --------
        float
            Z-coordinate (vertical position) in millimeters
            - Higher values = lower position (closer to ground)
            - Lower values = higher position (further from ground)
        
        Notes:
        ------
        Linear trajectory is used for vertical motion because:
        - Vertical displacement is small (20 mm)
        - Ground clearance requirements are less critical than horizontal precision
        - Simpler computation than cosine trajectory
        """
        return self.Z_ORIGIN - (self.LEG_STEP_HEIGHT * step / (self.STEP_COUNT-1))

    def get_coords(self):
        """
        get coords action coords calculation,
        fb: forward(1) or backward(-1)
        lr: left(1), middle(0) or right(-1)
        """
        origin_leg_coord = [[self.leg_origin[i] - self.LEG_ORIGINAL_Y_TABLE[i]
                             * self.section_length[i], self.Z_ORIGIN] for i in range(4)]
        leg_coords = []
        for section in range(self.SECTION_COUNT):
            for step in range(self.STEP_COUNT):
                if self.fb == 1:
                    raise_legs = self.LEG_RAISE_ORDER[section]
                else:
                    raise_legs = self.LEG_RAISE_ORDER[self.SECTION_COUNT - section - 1]
                leg_coord = []

                for i in range(4):
                    if i + 1 in raise_legs:
                        y = self.step_y_func(i, step)
                        z = self.step_z_func(step)
                    else:
                        y = origin_leg_coord[i][0] + \
                            self.step_down_length[i] * self.fb
                        z = self.Z_ORIGIN
                    leg_coord.append([y, z])
                origin_leg_coord = leg_coord
                leg_coords.append(leg_coord)
        return leg_coords


def test():

    from byte.byte import RobotDog
    dog = RobotDog(leg_pins=[1, 2, 3, 4, 5, 6, 7, 8],
                head_pins=[9, 10, 11], tail_pin=[12],
                )

    def pause():
        key = readchar.readkey()
        if key == readchar.key.CTRL_C or key in readchar.key.ESCAPE_SEQUENCES:
            import sys
            print('')
            sys.exit(0)

    forward = Trot(fb=Trot.FORWARD, lr=Trot.STRAIGHT)
    backward = Trot(fb=Trot.BACKWARD, lr=Trot.STRAIGHT)
    forward_left = Trot(fb=Trot.FORWARD, lr=Trot.LEFT)
    forward_right = Trot(fb=Trot.FORWARD, lr=Trot.RIGHT)
    backward_left = Trot(fb=Trot.BACKWARD, lr=Trot.LEFT)
    backward_right = Trot(fb=Trot.BACKWARD, lr=Trot.RIGHT)
    leg_coords = forward.get_coords()

    # try:
    while True:
        for leg_coord in leg_coords:
            # print(leg_coord)
            # dog.set_rpy(**rpy)
            # dog.set_pose(**pos)
            # dog.set_rpy(0, 0, 0, True)
            dog.set_legs(leg_coord)
            angles = dog.pose2legs_angle()
            dog.legs_simple_move(angles)
            # pause()
            delay(0.001)

    # finally:
    #     dog.close()


if __name__ == '__main__':
    test()
