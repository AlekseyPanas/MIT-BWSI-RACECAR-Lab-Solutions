"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 4A - IMU: Roll Prevention
"""

################################################################################
# Imports
################################################################################

import sys

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

import cv2 as cv
import numpy as np
import math


################################################################################
# Global variables
################################################################################

rc = racecar_core.create_racecar()

roll_displacement = 0
roll_reset_time = 0

################################################################################
# Functions
################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    print(
        ">> Lab 4A - IMU: Roll Prevention\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
    )


def update():
    global roll_displacement, roll_reset_time
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    # Calculate speed from triggers
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    # TODO (warmup): Prevent the car from turning too abruptly using the IMU
    ang_vel = rc.physics.get_angular_velocity()
    roll_vel = ang_vel[2]

    roll_displacement += roll_vel * rc.get_delta_time()

    if round(roll_vel, 3) < 0.003:
        roll_displacement *= 0.9
        print("test")

    print(roll_displacement)

    if roll_displacement > 0.2:
        angle = 0

    if abs(roll_vel) > 0.8 and roll_reset_time <= 0:
        roll_reset_time = 200

    if roll_reset_time > 0:
        roll_reset_time -= 1
        angle = 0
        speed = 0

    #if roll_displacement > 0.1:
    #    rc.drive.stop()

    roll_reset_time -= 1

    rc.drive.set_speed_angle(speed, angle)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
