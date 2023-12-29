"""
hmm
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2A - Color Image Line Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import random

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

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 120, 120), (120, 255, 255))  # The HSV range for the color blue
RED = ((0, 120, 120), (15, 255, 255))
GREEN = ((60, 120, 120), (80, 255, 255))

PRINT_MAP = {BLUE: "Blue", RED: "Red", GREEN: "Green"}

PRIORITY_ORDER = [BLUE, RED, GREEN] 
random.shuffle(PRIORITY_ORDER)
PRIORITY_ORDER = tuple(PRIORITY_ORDER)

# Perpendicular line detection threshold
PERP_THRES = 200
PERP_MIN_RATIO = 0.08

# 0 = follow, 1 = determine turn, 2 = turn
state = 0

### State = 1:
# chosen turn direction for car
turn = 0

# Set to True for full user control of racecar
TELEOP = False

# TODO (challenge 1): add HSV ranges for other colors

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour = None
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

########################################################################################
# Functions
########################################################################################
def check_perpendicular(contour):
    """
    Returns if the car is facing a horizontal line
    """
    mins = np.amin(contour, axis=0)
    maxs = np.amax(contour, axis=0)

    min_y = mins[0][1]
    max_y = maxs[0][1]
    min_x = mins[0][0]
    max_x = maxs[0][0]
            
    # Ratio of height to width of the detected contour
    contour_dim_ratio = (max_y - min_y) / (max_x - min_x)

    if contour_dim_ratio <= PERP_MIN_RATIO:
        if min_x <= rc.camera.get_width() / 2 - PERP_THRES and max_x >= rc.camera.get_width() / 2 + PERP_THRES:
            return True
    return False


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global contour
    global ONCE

    image = rc.camera.get_color_image()

    width = rc.camera.get_width()
    height = rc.camera.get_height() - CROP_FLOOR[0][0]

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # Finds the largest contour of each color in order in the priority list and breaks out once one of them is found
        contour = None
        found = False
        for col in PRIORITY_ORDER:
            contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, col[0], col[1]), MIN_CONTOUR_AREA)
            if contour is not None:
                found = True
                break
        if not found:
            contour = None

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Draws lines which show the horizontal threshold for detecting the line
        cv.line(image, (int(width / 2 - PERP_THRES), 0), (int(width / 2 - PERP_THRES), height), (0, 0, 255))
        cv.line(image, (int(width / 2 + PERP_THRES), 0), (int(width / 2 + PERP_THRES), height), (0, 0, 255))

        # Display the image to the screen
        rc.display.show_color_image(image)


def start():
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
    )
    [print("Priority", str(col + 1), PRINT_MAP[PRIORITY_ORDER[col]]) for col in range(len(PRIORITY_ORDER))]


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed, angle, turn, TELEOP, state

    # Search for contours in the current color image
    update_contour()

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        # Current implementation: bang-bang control (very choppy)
        # TODO (warmup): Implement a smoother way to follow the line
        angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1], 
                                                    rc.camera.get_width(), 
                                                    rc.camera.get_width() / 2 - 1, 1, 0) * 2, -1, 1)

    # Use the triggers to control the car's speed
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Gives full user control for debugging purposes
    if TELEOP:
        speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT) - rc.controller.get_trigger(rc.controller.Trigger.LEFT) 
        angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
        if speed < -1:
            speed = -1
        elif speed > 1:
            speed = 1
        if angle < -1:
            angle = -1
        elif angle > 1:
            angle = 1
        rc.drive.set_speed_angle(speed, angle)
    else:
        if state == 0:
            # Inverses direction when going backwards
            if speed < 0:
                angle = -angle
            rc.drive.set_speed_angle(speed, angle)
            if contour is not None and check_perpendicular(contour):
                state = 1
        if state == 1:
            turn = random.choice((-1, 1))
            state = 2
        if state == 2:
            rc.drive.set_speed_angle(speed, turn)
            if contour is not None and not check_perpendicular(contour):
                state = 0
    print("state:", state, "perp", check_perpendicular(contour) if contour is not None else "NONE")

    # Button A toggles teleop
    if rc.controller.was_pressed(rc.controller.Button.A):
        TELEOP = False if TELEOP else True


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if True:
        pass
    elif rc.camera.get_color_image() is None:
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
            print("".join(s) + " : area = " + str(contour_area), "Angle:", angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
