"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import Enum, IntEnum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################

class WallFollower:

    class State(IntEnum):
        pass

    class Const(Enum):
        TARGET_DIST = 30

    def __init__(self, manual_mode=False):
        self.__is_manual = manual_mode

        self.__right_wall = None
        self.__left_wall = None

    def update_lidar(self):
        ang = rc_utils.get_lidar_closest_point(rc.lidar.get_samples(), (40, 50))[0]
        self.__right_wall = rc_utils.get_lidar_average_distance((rc.lidar.get_samples() - 0.1) % 1000, ang)

        ang =rc_utils.get_lidar_closest_point(rc.lidar.get_samples(), (-50, -40))[0]
        self.__left_wall = rc_utils.get_lidar_average_distance((rc.lidar.get_samples() - 0.1) % 1000, ang)

    # Call this to run automatic slaloms
    def auto_control(self):
        #rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        #lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        #speed = rt - lt
        speed = 1

        diff = self.__right_wall - self.__left_wall
        angle = rc_utils.clamp(diff / 50, -1, 1)

        rc.drive.set_speed_angle(speed, angle)

        print("diff", diff, "ang",angle)

    # Call this to debug. Allows full user car control
    def user_control(self):
        # Gets speed using triggers
        rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        speed = rt - lt

        # Gets angle from x axis of left joystick
        angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

        # Sets speed and angle
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(angle, -1, 1))

    # Main overhead function. Runs state machine
    def run_follower(self):
        self.update_lidar()

        # Calls respective functions depending on if manual control was enabled
        if self.__is_manual:
            self.user_control()
        else:
            self.auto_control()

WALL = None

def start():
    global WALL
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    rc.drive.set_max_speed(0.25)

    WALL = WallFollower(manual_mode=False)

    # Print start message
    print(">> Lab 5B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Follow the wall to the right of the car without hitting anything.
    WALL.run_follower()


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
