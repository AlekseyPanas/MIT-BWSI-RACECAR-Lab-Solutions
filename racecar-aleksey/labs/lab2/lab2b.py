"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2B - Color Image Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import abc
import cv2 as cv
import numpy as np
import math
import PID
from enum import IntEnum

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30
# Contour area for 30cm
AREA_30CM = 26615
# Minimum contour area for the car to decide it needs to backup
BACKUP_AREA = 20000
# The speed below which the controller will change state
MIN_SPEED = 0.00007
# The area difference from target below which the state will change
AREA_RANGE = 100
# Min pixel distance from cone center to camera center to require realignmnet
MIN_DIST = 10
# How much of the cone can be cut off for the car to decide to back up
CUTOFF_THRESH = 100

DIST_PID = PID.PID(Kp=0.0001, Ki=0, Kd=0.0001)

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 100, 100), (20, 255, 255))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cutoff_length = 0

cur_state = None

########################################################################################
# Functions
########################################################################################

class State(IntEnum):
    SEARCH = 0
    BACK = 1
    ALIGN = 2
    DIST = 3
    STOP = 4


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center, contour_area, cutoff_length

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)
            #print(contour[:, :, 1].flatten().shape)
            y = contour[:, :, 1].flatten()
            x = contour[:, :, 0].flatten()

            # Indices of contour points which touch the bottom of the view
            indices = np.nonzero(y >= rc.camera.get_height() - 1)

            # If the cone contour touches the bottom, 2 points will exist there
            if y[y >= rc.camera.get_height() - 1].shape[0] >= 2:
                cutoff_length = np.amax(x[indices]) - np.amin(x[indices])
            else:
                cutoff_length = 0

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            cutoff_length = 0
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global cur_state

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Sets initial state
    cur_state = State.SEARCH

    # Print start message
    print(">> Lab 2B - Color Image Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cur_state

    # Search for contours in the current color image
    update_contour()

    # TODO: Park the car 30 cm away from the closest orange cone
    if cur_state == State.SEARCH:
        if contour_center is not None:
            cur_state = State.BACK
        else:
            rc.drive.set_speed_angle(0.5, 1)

    # Backs up if there isn't enough space to align
    elif cur_state == State.BACK:
        if contour_center is None:
            cur_state = State.SEARCH

        elif contour_area < BACKUP_AREA and cutoff_length < CUTOFF_THRESH:
            cur_state = State.ALIGN
            rc.drive.set_speed_angle(0, 0)

        else:
            speed = -0.25
            
            # Sets angle proportional to horizontal pixel distance from contour
            kP = 4
            angle = -rc_utils.clamp(rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1) * kP, -1, 1)

            rc.drive.set_speed_angle(speed, angle)

    elif cur_state == State.ALIGN:
        # If cone not found
        if contour_center is None:
            cur_state = State.SEARCH
        
        # If aligned successfully within the set range
        elif abs(contour_center[1] - rc.camera.get_width() / 2) <= MIN_DIST:
            cur_state = State.DIST

        # Align with proportional control
        else:
            # Sets angle proportional to horizontal pixel distance from contour
            kP = 4
            angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1) * kP, -1, 1)

            # Sets speed to a constant and sets motor angle and speed
            speed = 0.25 
            rc.drive.set_speed_angle(speed, angle)

    elif cur_state == State.DIST:
        print("MINY, THRES", cutoff_length, CUTOFF_THRESH)

        # If cone not found
        if contour_center is None:
            cur_state = State.SEARCH

        # Detects if full contour is not visible (needs backing up)
        elif cutoff_length >= CUTOFF_THRESH:
            cur_state = State.BACK

        # If speed is low and area is within target range, ends approach
        elif speed < MIN_SPEED and abs(contour_area - AREA_30CM) < AREA_RANGE:
            cur_state = State.STOP

        # If for any reason the car loses alignment in a somewhat significant way, realign
        elif abs(contour_center[1] - rc.camera.get_width() / 2) > MIN_DIST:
            cur_state = State.BACK

        else:
            # Approaches cone with Proportional and Derivative control
            speed = rc_utils.clamp(-DIST_PID(AREA_30CM - contour_area), -1, 1)

            # Proportional control with remap range
            # speed = rc_utils.clamp(rc_utils.remap_range(contour_area, AREA_30CM - 1, AREA_30CM, 1, 0) * kP, -1, 1)

            # Sets angle and speed
            angle = 0
            rc.drive.set_speed_angle(speed, angle)

    elif cur_state == State.STOP:
        rc.drive.set_speed_angle(0, 0)

    print("Area:", contour_area, "State:", cur_state.name, "Speed:", speed, "Setpoint:", AREA_30CM)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
