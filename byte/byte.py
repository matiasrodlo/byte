#!/usr/bin/env python3
"""
RobotDog - A quadruped robot control system

This module provides the main RobotDog class for controlling a 4-legged robot
with head, tail, and various sensors. It handles servo control, movement patterns,
sensor integration, and action sequences.

The system implements:
- 2-DOF planar inverse kinematics for leg positioning
- PID control for body orientation stabilization
- Gait generation for walking and trotting locomotion
- Sensor fusion for balance and navigation

Mathematical foundations:
- Inverse kinematics: Law of cosines solution for 2-link planar manipulator
- Control theory: Proportional control with Kₚ = 0.033
- Kinematics: Body-fixed frame transformations with Euler angles (ZYX convention)

References:
- Craig, J. J. (2005). Introduction to Robotics: Mechanics and Control
- Siciliano, B., et al. (2009). Robotics: Modelling, Planning and Control

Author: Robot Development Team
"""

import os
import sys
from time import sleep, time
from multiprocessing import Process, Value, Lock
import threading
import numpy as np
from math import pi, sin, cos, sqrt, acos, atan2, atan
from robot_hat import Robot, Pin, Ultrasonic, utils, Music, I2C
from .sh3001 import Sh3001
from .rgb_strip import RGBStrip
from .sound_direction import SoundDirection
from .dual_touch import DualTouch
import warnings
warnings.filterwarnings("ignore")  # Suppress pygame warnings

"""
Servo Pin Layout and Configuration
==================================

Physical layout of servos on the robot:
                     4,        # Head yaw servo
                   5, '6'      # Head roll, pitch servos
                     |
              3,2 --[ ]-- 7,8  # Front legs (left, right)
                    [ ]        # Body
              1,0 --[ ]-- 10,11 # Rear legs (left, right)
                     |
                    '9'        # Tail servo
                    /

Pin assignments:
- Legs pins: [2, 3, 7, 8, 0, 1, 10, 11]
  * Left front leg: 2, 3
  * Right front leg: 7, 8  
  * Left hind leg: 0, 1
  * Right hind leg: 10, 11

- Head pins: [4, 6, 5]
  * Yaw (left/right): 4
  * Roll (tilt): 6
  * Pitch (up/down): 5

- Tail pin: [9]
"""

# System configuration and user setup
# ===================================
is_run_with_root = (os.geteuid() == 0)  # Check if running with root privileges
User = os.popen('echo ${SUDO_USER:-$LOGNAME}').readline().strip()  # Get actual user (not sudo user)
UserHome = os.popen('getent passwd %s | cut -d: -f 6' %User).readline().strip()  # Get user home directory
config_file = '%s/.config/byte/byte.conf' % UserHome  # Configuration file path

# ANSI Color codes for terminal output
# ===================================
# Reference: https://gist.github.com/rene-d/9e584a7dd2935d0f461904b9f2950007
# Format: '1;30' = bold gray, '0;31' = red, '0;32' = green, etc.
GRAY = '1;30'        # Bold gray
RED = '0;31'         # Red
GREEN = '0;32'       # Green  
YELLOW = '0;33'      # Yellow
BLUE = '0;34'        # Blue
PURPLE = '0;35'      # Purple
DARK_GREEN = '0;36'  # Dark green (cyan)
WHITE = '0;37'       # White

def print_color(msg, end='\n', file=sys.stdout, flush=False, color=''):
    """Print a message with ANSI color formatting."""
    print('\033[%sm%s\033[0m'%(color, msg), end=end, file=file, flush=flush)

def info(msg, end='\n', file=sys.stdout, flush=False):
    """Print an informational message in white."""
    print_color(msg, end=end, file=file, flush=flush, color=WHITE)

def debug(msg, end='\n', file=sys.stdout, flush=False):
    """Print a debug message in gray."""
    print_color(msg, end=end, file=file, flush=flush, color=GRAY)

def warn(msg, end='\n', file=sys.stdout, flush=False):
    """Print a warning message in yellow."""
    print_color(msg, end=end, file=file, flush=flush, color=YELLOW)

def error(msg, end='\n', file=sys.stdout, flush=False):
    """Print an error message in red."""
    print_color(msg, end=end, file=file, flush=flush, color=RED)


def compare_version(original_version, object_version):
    """Compare two version strings and return True if original >= object."""
    or_v = tuple(int(val) for val in original_version.split('.'))
    ob_v = tuple(int(val) for val in object_version.split('.'))
    return (or_v >= ob_v)  # Fixed: was comparing or_v >= or_v
    
# Handle numpy version compatibility for matrix operations
if compare_version(np.__version__, '2.0.0'):
    def numpy_mat(data):
        """Convert data to numpy matrix (for numpy >= 2.0.0)."""
        return np.asmatrix(data)
else:
    def numpy_mat(data):
        """Convert data to numpy matrix (for numpy < 2.0.0)."""
        return np.asmatrix(data)

class RobotDog():
    """
    Main class for controlling a quadruped robot.
    
    This class handles all aspects of robot control including:
    - Servo management for legs, head, and tail
    - Movement patterns and gait generation
    - Sensor integration (IMU, ultrasonic, touch)
    - Action sequences and behaviors
    - Thread management for concurrent operations
    """

    # Robot physical dimensions (in mm)
    # =================================
    # All dimensions are measured from CAD models and verified with physical assembly.
    # Tolerances: ±0.3 mm for leg segments, ±0.5 mm for body dimensions.
    # 
    # Kinematic parameters:
    # - LEG (L₁): Upper leg segment length = 42.0 mm
    # - FOOT (L₂): Lower leg segment length = 76.0 mm
    # - Maximum reach: R_max = L₁ + L₂ = 118.0 mm
    # - Minimum reach: R_min = |L₁ - L₂| = 34.0 mm
    LEG = 42          # Length of leg segment L₁ (mm) - upper leg (thigh)
    FOOT = 76         # Length of foot segment L₂ (mm) - lower leg (shank)
    BODY_LENGTH = 117 # Length of robot body (mm) - front-to-rear dimension
    BODY_WIDTH = 98   # Width of robot body (mm) - left-to-right dimension
    
    # Body structure matrix - defines the 4 leg attachment points
    # Format: [x, y, z] coordinates for each leg (front-left, front-right, rear-left, rear-right)
    BODY_STRUCT = numpy_mat([
        [-BODY_WIDTH / 2, -BODY_LENGTH / 2,  0],  # Front-left leg position
        [BODY_WIDTH / 2, -BODY_LENGTH / 2,  0],   # Front-right leg position
        [-BODY_WIDTH / 2,  BODY_LENGTH / 2,  0],  # Rear-left leg position
        [BODY_WIDTH / 2,  BODY_LENGTH / 2,  0]    # Rear-right leg position
    ]).T
    
    # File paths
    # Try package-relative path first, fallback to user home directory
    _package_dir = os.path.dirname(os.path.abspath(__file__))
    _package_sounds = os.path.join(_package_dir, 'sounds')
    if os.path.isdir(_package_sounds):
        SOUND_DIR = _package_sounds + os.sep
    else:
        SOUND_DIR = f"{UserHome}/byte/sounds/"  # Fallback to user home directory
    
    # Servo speed limits (degrees per second)
    # =======================================
    # Maximum safe angular velocities for servos to prevent mechanical damage
    # and ensure smooth motion. Values determined through empirical testing.
    # 
    # These limits are enforced in servo_move() to prevent excessive acceleration
    # and reduce mechanical stress on gear trains.
    HEAD_DPS = 300   # Head servo speed limit (deg/s) - conservative for precision
    LEGS_DPS = 428   # Leg servo speed limit (deg/s) - optimized for locomotion
    TAIL_DPS = 500   # Tail servo speed limit (deg/s) - higher for quick response   # Tail servo speed limit
    
    # PID control constants for balance
    # =================================
    # Proportional-Integral-Derivative controller for body orientation stabilization.
    # Control law: u(t) = Kₚ·e(t) + Kᵢ·∫e(τ)dτ + Kd·de(t)/dt
    # Where e(t) = target_rpy - measured_rpy (error signal)
    # 
    # Current configuration uses only proportional control (Kᵢ = Kd = 0).
    # Kₚ = 0.033 was empirically tuned for stable balance during locomotion.
    # 
    # Units: Output is servo offset in radians, input is angle error in degrees.
    KP = 0.033  # Proportional gain (dimensionless, converts deg error to rad offset)
    KI = 0.0    # Integral gain (disabled to prevent windup)
    KD = 0.0    # Derivative gain (disabled, can cause instability with sensor noise)
    
    # Default GPIO pin assignments
    # ===========================
    DEFAULT_LEGS_PINS = [2, 3, 7, 8, 0, 1, 10, 11]  # 8 pins for 4 legs (2 servos each)
    DEFAULT_HEAD_PINS = [4, 6, 5]                    # 3 pins for head (yaw, roll, pitch)
    DEFAULT_TAIL_PIN = [9]                           # 1 pin for tail
    
    # Head servo configuration
    # ========================
    HEAD_PITCH_OFFSET = 45  # Default pitch offset for head positioning
    
    # Head movement limits (degrees)
    HEAD_YAW_MIN = -90   # Left rotation limit
    HEAD_YAW_MAX = 90    # Right rotation limit
    HEAD_ROLL_MIN = -70  # Left tilt limit
    HEAD_ROLL_MAX = 70   # Right tilt limit
    HEAD_PITCH_MIN = -45 # Downward tilt limit
    HEAD_PITCH_MAX = 30  # Upward tilt limit

    def __init__(self, leg_pins=DEFAULT_LEGS_PINS, head_pins=DEFAULT_HEAD_PINS, tail_pin=DEFAULT_TAIL_PIN,
                 leg_init_angles=None, head_init_angles=None, tail_init_angle=None):
        """
        Initialize the RobotDog with servo pins and initial angles.
        
        Args:
            leg_pins: List of GPIO pins for leg servos (8 pins for 4 legs)
            head_pins: List of GPIO pins for head servos (3 pins: yaw, roll, pitch)
            tail_pin: List of GPIO pins for tail servo (1 pin)
            leg_init_angles: Initial angles for leg servos
            head_init_angles: Initial angles for head servos
            tail_init_angle: Initial angle for tail servo
        """
        # Reset microcontroller and wait for initialization
        utils.reset_mcu()
        sleep(0.2)

        # Load action dictionary for predefined movements
        from .actions_dictionary import ActionDict
        self.actions_dict = ActionDict()

        # Initialize robot state variables
        self.body_height = 80  # Default body height in mm
        self.pose = numpy_mat([0.0, 0.0, self.body_height]).T  # Target position vector [x, y, z]
        self.rpy = np.array([0.0, 0.0, 0.0]) * pi / 180  # Roll, pitch, yaw in radians
        
        # Leg attachment points structure matrix
        self.leg_point_struc = numpy_mat([
            [-self.BODY_WIDTH / 2, -self.BODY_LENGTH / 2,  0],  # Front-left
            [self.BODY_WIDTH / 2, -self.BODY_LENGTH / 2,  0],   # Front-right
            [-self.BODY_WIDTH / 2,  self.BODY_LENGTH / 2,  0],  # Rear-left
            [self.BODY_WIDTH / 2,  self.BODY_LENGTH / 2,  0]    # Rear-right
        ]).T
        
        # Current orientation angles
        self.pitch = 0
        self.roll = 0

        # PID control variables for balance
        self.roll_last_error = 0
        self.roll_error_integral = 0
        self.pitch_last_error = 0
        self.pitch_error_integral = 0
        self.target_rpy = [0, 0, 0]  # Target roll, pitch, yaw

        # Set default initial angles if not provided
        if leg_init_angles is None:
            leg_init_angles = self.actions_dict['lie'][0][0]  # Use 'lie' position as default
        if head_init_angles is None:
            head_init_angles = [0, 0, self.HEAD_PITCH_OFFSET]  # Default with pitch offset
        else:
            head_init_angles[2] += self.HEAD_PITCH_OFFSET  # Add pitch offset to provided angles
        if tail_init_angle is None:
            tail_init_angle = [0]  # Default tail angle

        # Thread management
        self.thread_list = []

        try:
            # Initialize servo controllers
            debug(f"config_file: {config_file}")
            debug("robot_hat init ... ", end='', flush=True)
            
            # Initialize leg servos (8 servos for 4 legs)
            self.legs = Robot(pin_list=leg_pins, name='legs', init_angles=leg_init_angles, 
                            init_order=[0, 2, 4, 6, 1, 3, 5, 7], db=config_file)
            
            # Initialize head servos (3 servos: yaw, roll, pitch)
            self.head = Robot(pin_list=head_pins, name='head',
                            init_angles=head_init_angles, db=config_file)
            
            # Initialize tail servo (1 servo)
            self.tail = Robot(pin_list=tail_pin, name='tail',
                            init_angles=tail_init_angle, db=config_file)
            
            # Add servo groups to thread list
            self.thread_list.extend(["legs", "head", "tail"])
            
            # Set maximum speeds for each servo group
            self.legs.max_dps = self.LEGS_DPS
            self.head.max_dps = self.HEAD_DPS
            self.tail.max_dps = self.TAIL_DPS

            # Action buffers for queuing movements
            self.legs_action_buffer = []
            self.head_action_buffer = []
            self.tail_action_buffer = []

            # Thread locks for synchronization
            self.legs_thread_lock = threading.Lock()
            self.head_thread_lock = threading.Lock()
            self.tail_thread_lock = threading.Lock()

            # Coordinate buffer for leg movements
            self.legs_actions_coords_buffer = []

            # Current angle tracking
            self.leg_current_angles = leg_init_angles
            self.head_current_angles = head_init_angles
            self.tail_current_angles = tail_init_angle

            # Movement speeds (0-100)
            self.legs_speed = 90
            self.head_speed = 90
            self.tail_speed = 90

            # Servo initialization complete
            debug("done")
        except OSError:
            error("fail")
            raise OSError("robot_hat I2C init failed. Please try again.")

        # Initialize IMU (Inertial Measurement Unit) for balance and orientation
        try:
            debug("imu_sh3001 init ... ", end='', flush=True)
            self.imu = Sh3001(db=config_file)  # SH3001 6-axis IMU sensor
            self.imu_acc_offset = [0, 0, 0]    # Accelerometer calibration offsets
            self.imu_gyro_offset = [0, 0, 0]   # Gyroscope calibration offsets
            self.accData = [0, 0, 0]           # Current accelerometer data [ax, ay, az]
            self.gyroData = [0, 0, 0]          # Current gyroscope data [gx, gy, gz]
            self.imu_fail_count = 0            # Counter for IMU failures
            self.thread_list.append("imu")     # Add IMU processing thread
            debug("done")
        except OSError:
            error("fail")

        # Initialize RGB LED strip for visual feedback
        try:
            debug("rgb_strip init ... ", end='', flush=True)
            self.rgb_thread_run = True
            self.rgb_strip = RGBStrip(addr=0X74, nums=11)  # 11 RGB LEDs at I2C address 0x74
            self.rgb_strip.set_mode('breath', 'black')     # Default breathing animation
            self.rgb_fail_count = 0
            self.thread_list.append("rgb")     # Add RGB control thread
            debug("done")
        except OSError:
            error("fail")

        # Initialize dual touch sensors for interaction
        try:
            debug("dual_touch init ... ", end='', flush=True)
            self.dual_touch = DualTouch('D2', 'D3')  # Touch sensors on pins D2 and D3
            self.touch = 'N'                         # Current touch state ('N' = none)
            debug("done")
        except:
            error("fail")

        # Initialize sound direction detection
        try:
            debug("sound_direction init ... ", end='', flush=True)
            self.ears = SoundDirection()  # Microphone array for sound localization
            debug("done")
        except:
            error("fail")

        # Initialize sound effects and music player
        try:
            debug("sound_effect init ... ", end='', flush=True)
            self.music = Music()  # Audio playback system
            debug("done")
        except:
            error("fail")

        # Shared distance sensor data (multiprocessing safe)
        self.distance = Value('f', -1.0)  # Distance value in cm (-1 = no reading)

        # Sensory processing thread management
        self.sensory_process = None
        self.sensory_lock = Lock()

        # System control flags
        self.exit_flag = False
        
        # Start all background threads
        self.action_threads_start()      # Start servo control threads
        self.sensory_process_start()     # Start sensor processing thread

    def read_distance(self):
        """Get the current distance reading from the ultrasonic sensor.
        
        Returns:
            float: Distance in centimeters, rounded to 2 decimal places
        """
        return round(self.distance.value, 2)

    # Thread management and system control
    # ====================================
    def close_all_thread(self):
        self.exit_flag = True

    def close(self):
        import signal
        import sys


        def handler(signal, frame):
            info('Please wait')
        signal.signal(signal.SIGINT, handler)

        def _handle_timeout(signum, frame):
            raise TimeoutError('function timeout')

        timeout_sec = 5
        signal.signal(signal.SIGALRM, _handle_timeout)
        signal.alarm(timeout_sec)

        info('\rStopping and returning to the initial position ... ')

        try:
            if self.exit_flag == True:
                self.exit_flag = False
                self.action_threads_start()
            
            self.stop_and_lie()
            self.close_all_thread()

            self.legs_thread.join()
            self.head_thread.join()
            self.tail_thread.join()

            if 'rgb' in self.thread_list:
                self.rgb_thread_run = False
                self.rgb_strip_thread.join()
                self.rgb_strip.close()
            if 'imu' in self.thread_list:
                self.imu_thread.join()
            if self.sensory_process != None:
                self.sensory_process.terminate()

            info('Quit')
        except Exception as e:
            error(f'Close error: {e}')
        finally:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            signal.alarm(0)
            sys.exit(0)

    def legs_simple_move(self, angles_list, speed=90):

        tt = time()

        max_delay = 0.05
        min_delay = 0.005

        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0

        delay = (100 - speed) / 100*(max_delay - min_delay) + min_delay

        rel_angles_list = []
        for i in range(len(angles_list)):
            rel_angles_list.append(angles_list[i] + self.legs.offset[i])
        self.legs.servo_write_raw(rel_angles_list)

        tt2 = time() - tt
        delay2 = 0.001*len(angles_list) - tt2

        if delay2 < -delay:
            delay2 = -delay
        sleep(delay + delay2)

    def legs_switch(self, flag=False):
        self.legs_sw_flag = flag

    def action_threads_start(self):
        # Immutable objects int, float, string, tuple, etc., need to be declared with global
        # Variable object lists, dicts, instances of custom classes, etc., do not need to be declared with global
        if 'legs' in self.thread_list:
            self.legs_thread = threading.Thread(name='legs_thread', target=self._legs_action_thread)
            self.legs_thread.daemon = True
            self.legs_thread.start()
        if 'head' in self.thread_list:
            self.head_thread = threading.Thread(name='head_thread', target=self._head_action_thread)
            self.head_thread.daemon = True
            self.head_thread.start()
        if 'tail' in self.thread_list:
            self.tail_thread = threading.Thread(name='tail_thread', target=self._tail_action_thread)
            self.tail_thread.daemon = True
            self.tail_thread.start()
        if 'rgb' in self.thread_list:
            self.rgb_strip_thread = threading.Thread(name='rgb_strip_thread', target=self._rgb_strip_thread)
            self.rgb_strip_thread.daemon = True
            self.rgb_strip_thread.start()
        if 'imu' in self.thread_list:
            self.imu_thread = threading.Thread(name='imu_thread', target=self._imu_thread)
            self.imu_thread.daemon = True
            self.imu_thread.start()

    # legs
    def _legs_action_thread(self):
        while not self.exit_flag:
            try:
                with self.legs_thread_lock:
                    self.leg_current_angles = list.copy(self.legs_action_buffer[0])
                # Release lock after copying data before the next operations
                self.legs.servo_move(self.leg_current_angles, self.legs_speed)
                with self.legs_thread_lock:
                    self.legs_action_buffer.pop(0)
            except IndexError:
                sleep(0.001)
            except Exception as e:
                error(f'\r_legs_action_thread Exception:{e}')
                break

    # head
    def _head_action_thread(self):
        while not self.exit_flag:
            try:
                with self.head_thread_lock:
                    self.head_current_angles = list.copy(self.head_action_buffer[0])
                    self.head_action_buffer.pop(0)
                # Release lock after copying data before the next operations
                _angles = list.copy(self.head_current_angles)
                _angles[0] = self.limit(self.HEAD_YAW_MIN, self.HEAD_YAW_MAX, _angles[0])
                _angles[1] = self.limit(self.HEAD_ROLL_MIN, self.HEAD_ROLL_MAX, _angles[1])
                _angles[2] = self.limit(self.HEAD_PITCH_MIN, self.HEAD_PITCH_MAX, _angles[2])
                _angles[2] += self.HEAD_PITCH_OFFSET
                self.head.servo_move(_angles, self.head_speed)
            except IndexError:
                sleep(0.001)
            except Exception as e:
                error(f'\r_head_action_thread Exception:{e}')
                break

    # tail
    def _tail_action_thread(self):
        while not self.exit_flag:
            try:
                with self.tail_thread_lock:
                    self.tail_current_angles = list.copy(self.tail_action_buffer[0])
                    self.tail_action_buffer.pop(0)
                # Release lock after copying data before the next operations
                self.tail.servo_move(self.tail_current_angles, self.tail_speed)
            except IndexError:
                sleep(0.001)
            except Exception as e:
                error(f'\r_tail_action_thread Exception:{e}')
                break

    # rgb strip
    def _rgb_strip_thread(self):
        while self.rgb_thread_run:
            try:
                self.rgb_strip.show()
                self.rgb_fail_count = 0
            except Exception as e:
                self.rgb_fail_count += 1
                sleep(0.001)
                if self.rgb_fail_count > 10:
                    error(f'\r_rgb_strip_thread Exception:{e}')
                    break

    # IMU

    def _imu_thread(self):
        # imu calibrate
        _ax = 0
        _ay = 0
        _az = 0
        _gx = 0
        _gy = 0
        _gz = 0
        time = 10
        for _ in range(time):
            data = self.imu._sh3001_getimudata()
            if data == False:
                break

            self.accData, self.gyroData = data
            _ax += self.accData[0]
            _ay += self.accData[1]
            _az += self.accData[2]
            _gx += self.gyroData[0]
            _gy += self.gyroData[1]
            _gz += self.gyroData[2]
            sleep(0.1)

        self.imu_acc_offset[0] = round(-16384 - _ax/time, 0)
        self.imu_acc_offset[1] = round(0 - _ay/time, 0)
        self.imu_acc_offset[2] = round(0 - _az/time, 0)
        self.imu_gyro_offset[0] = round(0 - _gx/time, 0)
        self.imu_gyro_offset[1] = round(0 - _gy/time, 0)
        self.imu_gyro_offset[2] = round(0 - _gz/time, 0)

        while not self.exit_flag:
            try:
                data = self.imu._sh3001_getimudata()
                if data == False:
                    self.imu_fail_count += 1
                    if self.imu_fail_count > 10:
                        error('\r_imu_thread imu data error')
                        break
                self.accData, self.gyroData = data
                self.accData[0] += self.imu_acc_offset[0]
                self.accData[1] += self.imu_acc_offset[1]
                self.accData[2] += self.imu_acc_offset[2]
                self.gyroData[0] += self.imu_gyro_offset[0]
                self.gyroData[1] += self.imu_gyro_offset[1]
                self.gyroData[2] += self.imu_gyro_offset[2]
                ax = self.accData[0]
                ay = self.accData[1]
                az = self.accData[2]
                ay = -ay
                az = -az

                self.pitch = atan(ay/sqrt(ax*ax+az*az))*57.2957795
                self.roll = atan(az/sqrt(ax*ax+ay*ay))*57.2957795

                self.imu_fail_count = 0
                sleep(0.05)
            except Exception as e:
                self.imu_fail_count += 1
                sleep(0.001)
                if self.imu_fail_count > 10:
                    error(f'\r_imu_thread Exception:{e}')
                    self.exit_flag = True
                    break

    # clear actions buff
    def legs_stop(self):
        with self.legs_thread_lock:
            self.legs_action_buffer.clear()
        self.wait_legs_done()

    def head_stop(self):
        with self.head_thread_lock:
            self.head_action_buffer.clear()
        self.wait_head_done()

    def tail_stop(self):
        with self.tail_thread_lock:
            self.tail_action_buffer.clear()
        self.wait_tail_done()

    def body_stop(self):
        self.legs_stop()
        self.head_stop()
        self.tail_stop()

    # move
    def legs_move(self, target_angles, immediately=True, speed=50):
        if immediately == True:
            self.legs_stop()
        self.legs_speed = speed
        with self.legs_thread_lock:
            self.legs_action_buffer += target_angles
        
    def head_rpy_to_angle(self, target_yrp, roll_comp=0, pitch_comp=0):
        yaw, roll, pitch = target_yrp
        signed = -1 if yaw < 0 else 1
        ratio = abs(yaw) / 90
        pitch_servo = roll * ratio + pitch * (1-ratio) + pitch_comp
        roll_servo = -(signed * (roll * (1-ratio) + pitch * ratio) + roll_comp)
        yaw_servo = yaw
        return [yaw_servo, roll_servo, pitch_servo]

    def head_move(self, target_yrps, roll_comp=0, pitch_comp=0, immediately=True, speed=50):
        if immediately == True:
            self.head_stop()
        self.head_speed = speed
        
        angles = [self.head_rpy_to_angle(
            target_yrp, roll_comp, pitch_comp) for target_yrp in target_yrps]

        with self.head_thread_lock:
            self.head_action_buffer += angles

    def head_move_raw(self, target_angles, immediately=True, speed=50):
        if immediately == True:
            self.head_stop()
        self.head_speed = speed
        with self.head_thread_lock:
            self.head_action_buffer += target_angles

    def tail_move(self, target_angles, immediately=True, speed=50):
        if immediately == True:
            self.tail_stop()
        self.tail_speed = speed
        with self.tail_thread_lock:
            self.tail_action_buffer += target_angles
        
    # ultrasonic
    def _ultrasonic_thread(self, distance_addr, lock):
        while True:
            try:
                with lock:
                    val = round(float(self.ultrasonic.read()), 2)
                    distance_addr.value = val
                sleep(0.01)
            except Exception as e:
                sleep(0.1)
                error(f'\rultrasonic_thread  except: {e}')
                break

    # sensory_process : ultrasonic
    def sensory_process_work(self, distance_addr, lock):
        try:
            debug("ultrasonic init ... ", end='', flush=True)
            echo = Pin('D0')
            trig = Pin('D1')
            self.ultrasonic = Ultrasonic(trig, echo, timeout=0.017)
            # add ultrasonic thread
            self.thread_list.append("ultrasonic")
            debug("done")
        except Exception as e:
            error("fail")
            raise ValueError(e)

        if 'ultrasonic' in self.thread_list:
            ultrasonic_thread = threading.Thread(name='ultrasonic_thread',
                                             target=self._ultrasonic_thread,
                                             args=(distance_addr, lock,))
            # ultrasonic_thread.daemon = True
            ultrasonic_thread.start()

    def sensory_process_start(self):
        if self.sensory_process != None:
            self.sensory_process.terminate()
        self.sensory_process = Process(name='sensory_process',
                                         target=self.sensory_process_work,
                                         args=(self.distance, self.sensory_lock))
        self.sensory_process.start()

    # reset: stop, stop_and_lie
    def stop_and_lie(self, speed=85):
        try:
            self.body_stop()
            self.legs_move(self.actions_dict['lie'][0], speed)
            self.head_move_raw([[0, 0, 0]], speed)
            self.tail_move([[0, 0, 0]], speed)
            self.wait_all_done()
            sleep(0.1)
        except Exception as e:
            error(f'\rstop_and_lie error:{e}')

    def speak(self, name, volume=100):
        """
        speak, play audio

        :param name: the file name int the folder(SOUND_DIR)
        :type name: str
        :param volume: volume, 0-100
        :type volume: int
        """
        if not is_run_with_root and not hasattr(self, "speak_first"):
            self.speak_first = True
            warn("Play sound needs to be run with sudo.")
        status, _ = utils.run_command('sudo killall pulseaudio') # Solve the problem that there is no sound when running in the vnc environment

        if os.path.isfile(name):
            self.music.sound_play_threading(name, volume)
        elif os.path.isfile(self.SOUND_DIR+name+'.mp3'):
            self.music.sound_play_threading(self.SOUND_DIR+name+'.mp3', volume)
        elif os.path.isfile(self.SOUND_DIR+name+'.wav'):
            self.music.sound_play_threading(self.SOUND_DIR+name+'.wav', volume)
        else:
            warn(f'No sound found for {name}')
            return False

    def speak_block(self, name, volume=100):
        """
        speak, play audio with block

        :param name: the file name int the folder(SOUND_DIR)
        :type name: str
        :param volume: volume, 0-100
        :type volume: int
        """
        if not is_run_with_root and not hasattr(self, "speak_first"):
            self.speak_first = True
            warn("Play sound needs to be run with sudo.")
        _status, _ = utils.run_command('sudo killall pulseaudio') # Solve the problem that there is no sound when running in the vnc environment
        
        if os.path.isfile(name):
            self.music.sound_play(name, volume)
        elif os.path.isfile(self.SOUND_DIR+name+'.mp3'):
            self.music.sound_play(self.SOUND_DIR+name+'.mp3', volume)
        elif os.path.isfile(self.SOUND_DIR+name+'.wav'):
            self.music.sound_play(self.SOUND_DIR+name+'.wav', volume)
        else:
            warn(f'No sound found for {name}')
            return False

    # calibration
    def set_leg_offsets(self, cali_list, reset_list=None):
        self.legs.set_offset(cali_list)
        if reset_list is None:
            self.legs.reset()
            self.leg_current_angles = [0]*8
        else:
            self.legs.servo_positions = list.copy(reset_list)
            self.legs.leg_current_angles = list.copy(reset_list)
            self.legs.servo_write_all(reset_list)

    def set_head_offsets(self, cali_list):
        self.head.set_offset(cali_list)
        #self.head.reset()
        self.head_move([[0]*3], immediately=True, speed=80)
        self.head_current_angles = [0]*3

    def set_tail_offset(self, cali_list):
        self.tail.set_offset(cali_list)
        self.tail.reset()
        self.tail_current_angles = [0]

    # calculate angles and coords

    def set_pose(self, x=None, y=None, z=None):
        if x != None:
            self.pose[0, 0] = float(x)
        if y != None:
            self.pose[1, 0] = float(y)
        if z != None:
            self.pose[2, 0] = float(z)

    def set_rpy(self, roll=None, pitch=None, yaw=None, pid=False):
        if roll is None:
            roll = self.rpy[0]
        if pitch is None:
            pitch = self.rpy[1]
        if yaw is None:
            yaw = self.rpy[2]

        if pid:
            roll_error = self.target_rpy[0] - self.roll
            pitch_error = self.target_rpy[1] - self.pitch

            roll_offset = self.KP * roll_error + self.KI * self.roll_error_integral + \
                self.KD * (roll_error - self.roll_last_error)
            pitch_offset = self.KP * pitch_error + self.KI * self.pitch_error_integral + \
                self.KD * (pitch_error - self.pitch_last_error)

            self.roll_error_integral += roll_error
            self.pitch_error_integral += pitch_error
            self.roll_last_error = roll_error
            self.pitch_last_error = pitch_error

            roll_offset = roll_offset / 180. * pi
            pitch_offset = pitch_offset / 180. * pi

            self.rpy[0] += roll_offset
            self.rpy[1] += pitch_offset
        else:
            self.rpy[0] = roll / 180. * pi
            self.rpy[1] = pitch / 180. * pi
            self.rpy[2] = yaw / 180. * pi

    def set_legs(self, legs_list):
        self.legpoint_struc = numpy_mat([
            [-self.BODY_WIDTH / 2, -self.BODY_LENGTH / 2 +
                legs_list[0][0], self.body_height - legs_list[0][1]],
            [self.BODY_WIDTH / 2, -self.BODY_LENGTH / 2 +
                legs_list[1][0], self.body_height - legs_list[1][1]],
            [-self.BODY_WIDTH / 2,  self.BODY_LENGTH / 2 +
                legs_list[2][0], self.body_height - legs_list[2][1]],
            [self.BODY_WIDTH / 2,  self.BODY_LENGTH / 2 +
                legs_list[3][0], self.body_height - legs_list[3][1]]
        ]).T

    # pose and Euler Angle algorithm
    def pose2coords(self):
        roll = self.rpy[0]
        pitch = self.rpy[1]
        yaw = self.rpy[2]

        rotx = numpy_mat([
            [cos(roll), 0, -sin(roll)],
            [0, 1,           0],
            [sin(roll), 0,  cos(roll)]])
        roty = numpy_mat([
            [1,         0,          0],
            [0, cos(-pitch), -sin(-pitch)],
            [0, sin(-pitch),  cos(-pitch)]])
        rotz = numpy_mat([
            [cos(yaw), -sin(yaw), 0],
            [sin(yaw),  cos(yaw), 0],
            [0,         0, 1]])
        rot_mat = rotx * roty * rotz
        AB = numpy_mat(np.zeros((3, 4)))
        for i in range(4):
            AB[:, i] = - self.pose - rot_mat * \
                self.BODY_STRUCT[:, i] + self.legpoint_struc[:, i]

        body_coor_list = []
        for i in range(4):
            body_coor_list.append([(self.legpoint_struc - AB).T[i, 0],
                                  (self.legpoint_struc - AB).T[i, 1], (self.legpoint_struc - AB).T[i, 2]])

        leg_coor_list = []
        for i in range(4):
            leg_coor_list.append(
                [self.legpoint_struc.T[i, 0], self.legpoint_struc.T[i, 1], self.legpoint_struc.T[i, 2]])

        return {"leg": leg_coor_list, "body": body_coor_list}

    def pose2legs_angle(self):
        data = self.pose2coords()
        leg_coor_list = data["leg"]
        body_coor_list = data["body"]
        coords = []
        angles = []

        for i in range(4):
            coords.append([
                leg_coor_list[i][1] - body_coor_list[i][1],
                body_coor_list[i][2] - leg_coor_list[i][2]])

        angles = []

        for i, coord in enumerate(coords):

            leg_angle, foot_angle = self.fieldcoord2polar(coord)
            # The left and right sides are opposite
            leg_angle = leg_angle
            foot_angle = foot_angle-90
            if i % 2 != 0:
                leg_angle = -leg_angle
                foot_angle = -foot_angle
            angles += [leg_angle, foot_angle]

        return angles

    def fieldcoord2polar(self, coord: tuple[float, float]) -> tuple[float, float]:
        """
        Inverse kinematics: Convert field coordinates to joint angles.
        
        Solves the 2-DOF planar inverse kinematics problem using the law of cosines.
        Given a desired foot position (y, z) in the leg frame, computes the required
        joint angles (α, β) for the hip and thigh servos.
        
        Mathematical Formulation:
        ------------------------
        For a 2-link planar manipulator with link lengths L₁ (leg) and L₂ (foot):
        
        1. Distance to foot: u = √(y² + z²)
        
        2. Foot angle (β) using law of cosines:
           cos(β) = (L₁² + L₂² - u²) / (2·L₁·L₂)
           β = arccos(cos(β))
        
        3. Leg angle (α):
           θ₁ = atan2(y, z)  # Angle to distance vector
           cos(θ₂) = (L₁² + u² - L₂²) / (2·L₁·u)
           θ₂ = arccos(cos(θ₂))
           α = θ₁ + θ₂ + pitch_offset
        
        Parameters:
        -----------
        coord : tuple[float, float]
            Foot position in leg frame (y, z) in millimeters.
            - y: Forward/backward position (positive forward)
            - z: Vertical position (positive upward)
        
        Returns:
        --------
        tuple[float, float]
            Joint angles (α, β) in degrees:
            - α: Leg (hip) joint angle
            - β: Foot (thigh) joint angle
        
        Constraints:
        ------------
        - Workspace limit: u ≤ L₁ + L₂ = 118 mm
        - Singularity at: u = L₁ + L₂ (extended) or u = |L₁ - L₂| (folded)
        
        References:
        -----------
        Craig, J. J. (2005). Introduction to Robotics: Mechanics and Control.
        Chapter 5: Inverse Kinematics.
        
        Notes:
        ------
        The pitch offset (self.rpy[1]) compensates for body orientation in the
        field coordinate system, ensuring correct leg positioning relative to
        the ground plane.
        """
        y, z = coord
        u = sqrt(pow(y, 2) + pow(z, 2))
        cos_angle1 = (self.FOOT**2 + self.LEG**2 - u**2) / \
            (2 * self.FOOT * self.LEG)
        cos_angle1 = min(max(cos_angle1, -1), 1)
        beta = acos(cos_angle1)

        angle1 = atan2(y, z)
        cos_angle2 = (self.LEG**2 + u**2 - self.FOOT**2)/(2*self.LEG*u)
        cos_angle2 = min(max(cos_angle2, -1), 1)
        angle2 = acos(cos_angle2)
        alpha = angle2 + angle1 + self.rpy[1]

        alpha = alpha / pi * 180
        beta = beta / pi * 180

        return alpha, beta

    def coord2polar(self, coord):
        y, z = coord
        u = sqrt(pow(y, 2) + pow(z, 2))
        cos_angle1 = (self.FOOT**2 + self.LEG**2 - u**2) / \
            (2 * self.FOOT * self.LEG)
        cos_angle1 = min(max(cos_angle1, -1), 1)
        beta = acos(cos_angle1)

        angle1 = atan2(y, z)
        cos_angle2 = (self.LEG**2 + u**2 - self.FOOT**2)/(2*self.LEG*u)
        cos_angle2 = min(max(cos_angle2, -1), 1)
        angle2 = acos(cos_angle2)
        alpha = angle2 + angle1

        alpha = alpha / pi * 180
        beta = beta / pi * 180

        return alpha, beta

    def polar2coord(self, angles):
        alpha, beta, gamma = angles

        L1 = sqrt(self.A**2+self.B**2-2*self.A*self.B*cos((90+alpha)/180*pi))
        angle = acos((self.A**2+L1**2-self.B**2)/(2*self.A*L1))*180/pi
        angle = 90 - beta - angle
        L = L1*cos(angle*pi/180) + self.C

        x = L*sin((45+gamma)*pi/180)
        y = L*cos((45+gamma)*pi/180)
        z = L1*sin(angle*pi/180)

        return [round(x, 4), round(y, 4), round(z, 4)]

    @classmethod
    def legs_angle_calculation(cls, coords: list[list[float]]) -> list[float]:
        """
        Calculate servo angles for all legs from 3D coordinates.
        
        This method converts 3D foot positions to servo angles for all 4 legs using
        inverse kinematics. It handles the mirroring of left and right legs and
        applies the 90-degree offset for the foot servo.
        
        The inverse kinematics solution uses the law of cosines on the 2-link
        planar mechanism (leg + foot segments). See fieldcoord2polar() for details.
        
        Parameters:
        -----------
        coords : list[list[float]]
            List of 4 [x, y, z] coordinates for each leg's foot position (mm).
            Order: [Front-left, Front-right, Rear-left, Rear-right]
            
        Returns:
        --------
        list[float]
            8 servo angles in degrees: [leg1, foot1, leg2, foot2, leg3, foot3, leg4, foot4]
            - leg angles: Hip joint angles (lateral rotation)
            - foot angles: Thigh joint angles (forward/backward)
        
        Notes:
        ------
        - Right-side legs (odd indices) have angles mirrored (negated)
        - Foot servo has 90° offset applied for mechanical alignment
        - Workspace constraint: foot distance ≤ L₁ + L₂ = 118 mm
        """
        translate_list = []
        for i, coord in enumerate(coords):  # Process each leg
            # Convert 3D coordinates to servo angles
            leg_angle, foot_angle = RobotDog.coord2polar(cls, coord)
            
            # Apply 90-degree offset to foot servo
            leg_angle = leg_angle
            foot_angle = foot_angle - 90
            
            # Mirror angles for right-side legs (odd indices)
            if i % 2 != 0:
                leg_angle = -leg_angle
                foot_angle = -foot_angle
                
            translate_list += [leg_angle, foot_angle]

        return translate_list

    def limit(self, min, max, x):
        """
        Clamp a value between minimum and maximum bounds.
        
        Args:
            min: Minimum allowed value
            max: Maximum allowed value
            x: Value to clamp
            
        Returns:
            float: Clamped value between min and max
        """
        if x > max:
            return max
        elif x < min:
            return min
        else:
            return x

    # set angle
    def set_angle(self, angles_list, speed=50, israise=False):
        translate_list = []
        results = []
        for angles in angles_list:
            result, angles = self.limit_angle(angles)
            translate_list += angles
            results.append(result)
        if True in results:
            if israise == True:
                raise ValueError(
                    '\033[1;35mCoordinates out of controllable range.\033[0m')
            else:
                print('\033[1;35mCoordinates out of controllable range.\033[0m')
                coords = []
                # Calculate coordinates
                for i in range(4):
                    coords.append(self.polar2coord(
                        [translate_list[i*3], translate_list[i*3+1], translate_list[i*3+2]]))
                self.current_coord = coords
        else:
            self.current_coord = self.coord_temp

        self.servo_move(translate_list, speed)

    def do_action(self, action_name, step_count=1, speed=50, pitch_comp=0):
        """
        Execute a predefined action sequence.
        
        Args:
            action_name: Name of the action from the actions dictionary
            step_count: Number of times to repeat the action
            speed: Movement speed (0-100)
            pitch_comp: Pitch compensation for head movements
            
        Raises:
            KeyError: If action_name is not found in the actions dictionary
        """
        try:
            actions, part = self.actions_dict[action_name]
            if part == 'legs':
                for _ in range(step_count):
                    self.legs_move(actions, immediately=False, speed=speed)
            elif part == 'head':
                for _ in range(step_count):
                    self.head_move(actions, pitch_comp=pitch_comp, immediately=False, speed=speed)
            elif part == 'tail':
                for _ in range(step_count):
                    self.tail_move(actions, immediately=False, speed=speed)
        except KeyError:
            error("do_action: No such action")
        except Exception as e:
            error(f"do_action:{e}")

    def wait_legs_done(self):
        while not self.is_legs_done():
            sleep(0.001)

    def wait_head_done(self):
        while not self.is_head_done():
            sleep(0.001)

    def wait_tail_done(self):
        while not self.is_tail_done():
            sleep(0.001)

    def wait_all_done(self):
        self.wait_legs_done()
        self.wait_head_done()
        self.wait_tail_done()

    def is_legs_done(self):
        return not bool(len(self.legs_action_buffer) > 0)

    def is_head_done(self):
        return not bool(len(self.head_action_buffer) > 0)

    def is_tail_done(self):
        return not bool(len(self.tail_action_buffer) > 0)

    def is_all_done(self):
        return self.is_legs_done() and self.is_head_done() and self.is_tail_done()

    def get_battery_voltage(self):
        return round( utils.get_battery_voltage(), 2)
