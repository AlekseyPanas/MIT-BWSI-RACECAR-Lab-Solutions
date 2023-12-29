"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
timer = 0
# Time to turn 90 degrees
TIME_90 = 2.5
# Time length for each side of the square
TIME_FORWARD = 2
# Time to finish full circle at 0.5 speed and 1 turn angle
TIME_CIRCLE = 23

# 1=square, -1=default, 2=circle, 3=figure8, 4=custom
rc_state = -1

# Used to complete square path
square_count = 0
# Used for tracking zig zags
zigzag_count = 0
# How many zig zags to do
ZIGZAG_QUANT = 5
# f = forward, t = turn
mode = "f"

# Put any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    #rc.drive.set_max_speed(1)

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does

    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = drive in a circle\n"
        "   B button = drive in a square\n"
        "   X button = drive in a figure eight\n"
        "   Y button = drive zigzag\n"
    )


def update():
    global timer, square_count, rc_state, mode, zigzag_count
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO (warmup): Implement acceleration and steering
    if rc_state == -1:
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

    # TODO (main challenge): Drive in a circle
    if rc.controller.was_pressed(rc.controller.Button.A) and rc_state == -1:
        rc_state = 2
        timer=0

    if rc_state == 2:
        print("Driving in a circle...")
        rc.drive.set_speed_angle(0.5, 1)
        if timer > TIME_CIRCLE:
            rc_state=-1
            rc.drive.stop()
            print("Circle finished")

        timer += rc.get_delta_time()
            
    # TODO (main challenge): Drive in a square when the B button is pressed
    if rc.controller.was_pressed(rc.controller.Button.B) and rc_state == -1:
        rc_state = 1
        timer = 0
        square_count = TIME_FORWARD + 1
        mode = "f"
        print("Starting square path")

    if rc_state == 1:
        print(timer, mode, square_count)
        if timer < square_count and mode == "f":
            rc.drive.set_speed_angle(0.5, 0)
            print("driving")
        elif mode == "f":
            square_count += TIME_90
            mode = "t"
            
        if timer < square_count and mode == "t":
            rc.drive.set_speed_angle(0.5, 1)
            print("turning")
        elif mode == "t":
            square_count += TIME_FORWARD
            mode = "f"

        if timer > (4 * TIME_FORWARD) + (4 * TIME_90):
            rc_state = -1
            rc.drive.stop()
            print("Ended square path")
        print()
            
        

    # TODO (main challenge): Drive in a figure eight when the X button is pressed
    #if rc.controller.was _pressed(rc.controller.Button.X) and rc_state==-1:
    if rc.controller.was_pressed(rc.controller.Button.X) and rc_state == -1:
        rc_state = 3
        timer = 0

    if rc_state == 3:
        print("Driving in a figure 8...")
        if timer <= TIME_CIRCLE:
            rc.drive.set_speed_angle(0.5, 1)
        elif timer <= 2 * TIME_CIRCLE:
            rc.drive.set_speed_angle(0.5, -1)
        else:
            rc_state=-1
            rc.drive.stop()
            print("Figure finished")

        timer += rc.get_delta_time()

    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # is pressed
    if rc.controller.was_pressed(rc.controller.Button.Y) and rc_state == -1:
        rc_state = 4
        timer = 0
        zigzag_count = 0

    if rc_state == 4:
        print("Driving zigzag...")
        if timer <= TIME_FORWARD:
            rc.drive.set_speed_angle(0.5, 0)
        elif timer <= TIME_FORWARD + (TIME_90 / 2):
            rc.drive.set_speed_angle(0.5, 1)
        elif timer <= (2 * TIME_FORWARD) + (TIME_90 / 2):
            rc.drive.set_speed_angle(0.5, 0)
        elif timer <= (2 * TIME_FORWARD) + (TIME_90):
            rc.drive.set_speed_angle(0.5, -1)
        else:
            zigzag_count += 1
            timer = 0
            print(zigzag_count)
        if zigzag_count > ZIGZAG_QUANT:
            rc_state=-1
            rc.drive.stop()
            print("Zigzag finished")

        timer += rc.get_delta_time()

    timer += rc.get_delta_time()

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
