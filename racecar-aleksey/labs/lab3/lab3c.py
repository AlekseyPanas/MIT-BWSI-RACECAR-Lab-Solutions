"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import PID
from enum import IntEnum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

PERP_PID = PID.PID(Kp=0.05, Ki=0, Kd=0.01)
PERP_SET = 0

DIST_PID = PID.PID(Kp=0.02, Ki=0, Kd=0.01)
DIST_SET = 20

KERNEL_SIZE = 5
TRIM = 100

ALIGN_MIN_DIST = 40

ANGLE_THRESH = 0.05
DIFF_THRESH = 0.3

class State:
    ALIGN_BACK = 0
    ALIGN_FORWARD = 1
    APPROACH = 2

cur_state = State.ALIGN_BACK

# Add any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_state

    # TODO: Park the car 20 cm away from the closest wall with the car directly facing
    # the wall
    dep_img = cv.GaussianBlur((rc.camera.get_depth_image() - 0.01) % 10000, (KERNEL_SIZE, KERNEL_SIZE), 0) 

    if cur_state == State.ALIGN_BACK or cur_state == State.ALIGN_FORWARD:
        arr = dep_img[rc.camera.get_height() // 2 - 5 : rc.camera.get_height() // 2 + 5, :]

        arr = arr[np.where(arr < 2000)]
        arr = arr[0 + TRIM : arr.size - TRIM + 1]
        if arr.size >= 2:
            left_side = np.sum(arr[0 : 10]) / 10
            right_side = np.sum(arr[arr.size - 10 : arr.size]) / 10
            diff = left_side - right_side
        else:
            diff = None

        center_dist = rc_utils.get_pixel_average_distance(dep_img, (rc.camera.get_height() // 2, rc.camera.get_width() // 2))

        if cur_state == State.ALIGN_BACK:
            speed = -0.5
            angle = PERP_PID(diff)
            if center_dist > ALIGN_MIN_DIST * 3:
                cur_state = State.ALIGN_FORWARD

        elif cur_state == State.ALIGN_FORWARD:
            speed = 0.5
            angle = -PERP_PID(diff)
            if center_dist < ALIGN_MIN_DIST:
                cur_state = State.ALIGN_BACK
                

        if abs(angle) < ANGLE_THRESH and abs(diff) < DIFF_THRESH:
            cur_state = State.APPROACH

    elif cur_state == State.APPROACH:
        center_dist = rc_utils.get_pixel_average_distance(dep_img, (rc.camera.get_height() // 2, rc.camera.get_width() // 2))

        angle = 0
        speed = DIST_PID(DIST_SET - center_dist)

        diff = "HEHEHE"

    rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(angle, -1, 1))

    print("diff", diff, "state", cur_state, "angle", angle)



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
