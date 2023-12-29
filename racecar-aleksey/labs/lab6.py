"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 6 - Sensor Fusion
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import Enum, IntEnum

sys.path.insert(0, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

########################################################################################
# Functions
########################################################################################


class Fusion:

    class State(IntEnum):
        pass

    class Const(Enum):
        SPEED_REMAP = (0.5, 0, 0.2, 1)

        LIDAR_MAX_DIFF = 20
        DEPTH_MAX_DIFF = 20

        VARIANCE_ALPHA = 0.2

    def __init__(self, manual_mode=False):
        self.__is_manual = manual_mode

        self.__lidar_scan = None
        self.__depth_image = None
        self.__acceleration = None

        self.__lidar_speed = None
        self.__depth_speed = None
        self.__imu_speed = 0

        # User to track previous values for distance calculations
        self.__lidar_prev = None
        self.__depth_prev = None

        # Variance running averages
        self.__var_run_avg = [0, 0, 0]

    def update_data(self):
        self.__lidar_scan = (rc.lidar.get_samples() - 0.1) % 1000
        self.__depth_image = (rc.camera.get_depth_image() - 0.1) % 1000
        self.__acceleration = rc.physics.get_linear_acceleration()

    def update_speed_lidar(self):
        # Gets average distance to nearest point between -20 and 20 degrees
        dist = rc_utils.get_lidar_average_distance(self.__lidar_scan, rc_utils.get_lidar_closest_point(self.__lidar_scan, (-20, 20))[0])

        if self.__lidar_prev is not None:
            # Checks if the difference in distance isn't totally whack
            diff = dist - self.__lidar_prev
            if diff <= self.Const.LIDAR_MAX_DIFF.value:
                self.__lidar_speed = diff / rc.get_delta_time()
        else:
            self.__lidar_speed = 0

        self.__lidar_prev = dist


    def update_speed_depth(self):
        # Gets average distance to point at center of depth image
        dist = rc_utils.get_pixel_average_distance(self.__depth_image, (rc.camera.get_height() // 2, 
                                                                        rc.camera.get_width() // 2))  
        # self.__depth_image[0: self.__depth_image.get_height() * 2 // 3, :]

        if self.__depth_prev is not None:
            # Checks if the difference in distance isn't totally whack
            diff = dist - self.__depth_prev
            if diff <= self.Const.DEPTH_MAX_DIFF.value:
                # Sets speed
                self.__depth_speed =  diff / rc.get_delta_time()
        else:
            self.__depth_speed = 0

        self.__depth_prev = dist

    def update_speed_imu(self):
        # Updates speed with current acceleration and time change
        # Note: This measurement will become inaccurate over time
        self.__imu_speed += self.__acceleration[2] * rc.get_delta_time()

    def get_final_velocity(self) -> float:
        self.update_speed_depth()
        self.update_speed_lidar()
        self.update_speed_imu()

        # Get all speeds that aren't evaluated to None
        # list contains (idx, speed)
        speeds = [x for x in enumerate((self.__imu_speed, self.__lidar_speed, self.__depth_speed)) if x[1] is not None]

        # If at least one speed was found..
        if len(speeds):
            length = len(speeds)
            avg = sum([speed for _, speed in speeds]) / length

            # Updates the variance of each speed measurement to the running avg
            for idx, speed in speeds:
                self.__var_run_avg[idx] += (((speed - avg) ** 2) - self.__var_run_avg[idx]) * self.Const.VARIANCE_ALPHA.value
            inverse_variances = [var ** -1 for var in self.__var_run_avg]

            var_sum = sum(inverse_variances[idx] for idx, _ in speeds)

            # Applies weights to each speed measurement according to their variance and finds the average speed
            final_velocity = sum([(inverse_variances[idx] / var_sum) * speed for idx, speed in speeds])

        else:
            # Ideally, you don't want to find yourself here
            print("NO Speed! You Blind!")
            final_velocity = 0
        
        return final_velocity

    # Call this to run automatic course completion
    def auto_control(self):
        # Gets current speed
        vel = self.get_final_velocity()

        print("SPEEDS----", "imu:", round(self.__imu_speed, 2), 
                            "lidar:", round(self.__lidar_speed, 2), 
                            "depth:", round(self.__depth_speed, 2), round(vel, 2))
        print("VARS------", [round(x, 2) for x in self.__var_run_avg])

        # Maps the current speed to the desired amount of acceleration to apply
        speed = rc_utils.remap_range(vel, *self.Const.SPEED_REMAP.value)
        angle = 0

        # Go Vroom
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(angle, -1, 1))

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

    # Main overhead function
    def run_fusion(self):
        self.update_data()

        # Calls respective functions depending on if manual control was enabled
        if self.__is_manual:
            self.user_control()
        else:
            self.auto_control()


FUSION = None


def start():
    global FUSION

    """
    This function is run once every time the start button is pressed
    """

    FUSION = Fusion(manual_mode=False)

    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 6 - Sensor Fusion")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Use the triggers to control the car's speed
    '''rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]'''

    FUSION.run_fusion()

    # TODO: Estimate the car's speed with at least 3 unique methods

    # TODO: Fuse these sources into a single velocity measurement

    # TODO: Prevent the car from traveling over 0.5 m/s

    #rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
