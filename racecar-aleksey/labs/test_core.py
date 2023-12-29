"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

A simple program which can be used to manually test racecar_core functionality.
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

max_speed = 0
update_slow_time = 0
show_triggers = False
show_joysticks = False

contour_area = 0
contour_center = None

TARGET_COLOR = ((60, 60, 60), (80, 255, 255))
MIN_CONTOUR_AREA = 100

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global max_speed
    global update_slow_time
    global show_triggers
    global show_joysticks

    print("Start function called")
    max_speed = 1
    update_slow_time = 0.5
    show_triggers = False
    show_joysticks = False

    rc.set_update_slow_time(update_slow_time)
    rc.drive.set_max_speed(max_speed)
    rc.drive.stop()

    # Print start message
    print(
        ">> Test Core: A testing program for the racecar_core library.\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   Left bumper = decrease max speed\n"
        "   Right bumper = increase max speed\n"
        "   Left joystick click = print trigger values\n"
        "   Right joystick click = print joystick values\n"
        "   A button = Display color image\n"
        "   B button = Display depth image\n"
        "   X button = Display lidar data\n"
        "   Y button = Display IMU data\n"
    )


def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center, contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, TARGET_COLOR[0], TARGET_COLOR[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            cutoff_length = 0
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global max_speed
    global update_slow_time
    global show_triggers
    global show_joysticks

    update_contour()

    # Check if each button was_pressed or was_released
    for button in rc.controller.Button:
        if rc.controller.was_pressed(button):
            print("Button {} was pressed".format(button.name))
        if rc.controller.was_released(button):
            print("Button {} was released".format(button.name))

    # Click left and right joystick to toggle showing trigger and joystick values
    left_trigger = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    right_trigger = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    left_joystick = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    right_joystick = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)

    if rc.controller.was_pressed(rc.controller.Button.LJOY):
        show_triggers = not show_triggers

    if rc.controller.was_pressed(rc.controller.Button.RJOY):
        show_joysticks = not show_joysticks

    if show_triggers:
        print("Left trigger: {}; Right trigger: {}".format(left_trigger, right_trigger))

    if show_joysticks:
        print(
            "Left joystick: {}; Right joystick: {}".format(
                left_joystick, right_joystick
            )
        )

    # Use triggers and left joystick to control car (like default drive)
    rc.drive.set_speed_angle(right_trigger - left_trigger, left_joystick[0])

    # Change max speed and update_slow time when the bumper is pressed
    if rc.controller.was_pressed(rc.controller.Button.LB):
        max_speed = max(1 / 16, max_speed / 2)
        rc.drive.set_max_speed(max_speed)
        update_slow_time *= 2
        rc.set_update_slow_time(update_slow_time)
        print("max_speed set to {}".format(max_speed))
        print("update_slow_time set to {} seconds".format(update_slow_time))
    if rc.controller.was_pressed(rc.controller.Button.RB):
        max_speed = min(1, max_speed * 2)
        rc.drive.set_max_speed(max_speed)
        update_slow_time /= 2
        rc.set_update_slow_time(update_slow_time)
        print("max_speed set to {}".format(max_speed))
        print("update_slow_time set to {} seconds".format(update_slow_time))

    # Capture and display color images when the A button is down
    if rc.controller.is_down(rc.controller.Button.A):
        rc.display.show_color_image(rc.camera.get_color_image())

    # Capture and display depth images when the B button is down
    elif rc.controller.is_down(rc.controller.Button.B):
        depth_image = rc.camera.get_depth_image()
        rc.display.show_depth_image(depth_image)
        print(
            "Depth center distance: {:.2f} cm".format(
                rc_utils.get_depth_image_center_distance(depth_image)
            )
        )

    # Capture and display Lidar data when the X button is down
    elif rc.controller.is_down(rc.controller.Button.X):
        lidar = rc.lidar.get_samples()
        rc.display.show_lidar(lidar)
        print(
            "LIDAR forward distance: {:.2f} cm".format(
                rc_utils.get_lidar_average_distance(lidar, 0)
            )
        )

    # Show IMU data when the Y button is pressed
    if rc.controller.is_down(rc.controller.Button.Y):
        a = rc.physics.get_linear_acceleration()
        w = rc.physics.get_angular_velocity()
        print(
            "Linear acceleration: ({:5.2f},{:5.2f},{:5.2f}); ".format(a[0], a[1], a[2])
            + "Angular velocity: ({:5.2f},{:5.2f},{:5.2f})".format(w[0], w[1], w[2])
        )


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Check if each button is_down
    for button in rc.controller.Button:
        if rc.controller.is_down(button):
            print("Button {} is down".format(button.name))

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
            #print("".join(s) + " : area = " + str(contour_area), "Angle:", angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
