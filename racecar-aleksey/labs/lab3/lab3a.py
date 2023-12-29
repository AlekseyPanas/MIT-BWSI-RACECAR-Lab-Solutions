"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
STOP_DISTANCE = 75
CHECK_OFFSET = 150
KERNEL_SIZE = 5

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
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Right bumper = override safety stop\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = print current speed and angle\n"
        "   B button = print the distance at the center of the depth image"
    )


def is_ramp(img):
    arr = img[0 : rc.camera.get_height() // 2, rc.camera.get_width() // 2]

    arr = arr[np.where(arr < STOP_DISTANCE * 3)]
    if arr.size > 5:
        diff = np.average(arr) - np.amin(arr)
    else:
        diff = 0

    THRESH = 10

    return diff > THRESH


def is_ledge(img):
    pixel = img[rc.camera.get_height() * 5 // 6, rc.camera.get_width() // 2]
    THRESH = 150
    print(pixel)

    return THRESH < pixel < 1000


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Calculate the distance of the object directly in front of the car
    #depth_image = (rc.camera.get_depth_image()[0 : rc.camera.get_height() * 2 // 3, :] - 0.01) % 10000
    depth_image = cv.GaussianBlur((rc.camera.get_depth_image() - 0.01) % 10000, (KERNEL_SIZE, KERNEL_SIZE), 0)

    center_distance = rc_utils.get_depth_image_center_distance(depth_image)
    depth_slice = depth_image[rc.camera.get_height()//2, rc.camera.get_width() // 2 - CHECK_OFFSET : rc.camera.get_width() // 2 + CHECK_OFFSET]
    min_dist = np.amin(depth_slice)
    rc_utils.draw_circle(depth_image, (rc.camera.get_height()//2, rc.camera.get_width()//2), color=(255, 0, 0))

    print("cent", center_distance)

    # TODO (warmup): Prevent forward movement if the car is about to hit something.
    # Allow the user to override safety stop by holding the right bumper.
    if not rc.controller.is_down(rc.controller.Button.RB):
        if min_dist < STOP_DISTANCE:
            if not is_ramp(depth_image):
                speed = -1
        if is_ledge(depth_image):
            speed = -1 

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Center distance:", center_distance)

    # Display the current depth image
    rc.display.show_depth_image(depth_image)

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.

    # TODO (stretch goal): Tune safety stop so that the car is still able to drive up
    # and down gentle ramps.
    # Hint: You may need to check distance at multiple points.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
