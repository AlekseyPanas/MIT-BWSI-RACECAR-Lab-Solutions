"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020
​
Final Challenge - Time Trials

69.4 degrees (FOV)
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import Enum, IntEnum
from nptyping import NDArray
from typing import Any, Tuple, List, Optional

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
import ar

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

########################################################################################
# Functions
########################################################################################

"""
.___  ___.      ___       __  .__   __. 
|   \/   |     /   \     |  | |  \ |  | 
|  \  /  |    /  ^  \    |  | |   \|  | 
|  |\/|  |   /  /_\  \   |  | |  . `  | 
|  |  |  |  /  _____  \  |  | |  |\   | 
|__|  |__| /__/     \__\ |__| |__| \__|
"""

class TimeTrials:

    class Phase(IntEnum):
        LINE_FOLLOW = 0
        LANE_FOLLOW = 1
        SLALOM = 2
        WALL = 3

    class Const(Enum):
        SLALOM_AR_MIN_DIST = 70
        SLALOM_AR_ID = 2

    def __init__(self, manual_mode=False):
        self.__is_manual = manual_mode

        self.phases = ( Phase0(), Phase1(), Phase2(), Phase3() )
        self.cur_phase = None

        self.image = None
        self.hsv = None
        self.depth_image = None
        self.lidar = None

        self.min_dist_lane_ar = 9999

    def update_images(self):
        self.image = rc.camera.get_color_image()
        self.hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        self.depth_image = (rc.camera.get_depth_image() - 0.1) % 10000
        self.lidar_image = (rc.lidar.get_samples() - 0.1) % 10000

    def phase_management(self):
        # Init first state
        if self.cur_phase is None:
            self.cur_phase = self.Phase.LINE_FOLLOW
            self.phases[0].set_line_priority()
        
        elif self.cur_phase == self.Phase.LINE_FOLLOW:
            print(self.phases[0].priority)

            out = self.phases[1].set_colors()

            if out[2] is not None and out[2] < self.min_dist_lane_ar:
                self.min_dist_lane_ar = out[2]

            # Changes crop
            if out[1]:
                self.phases[0].CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

                # Sets colors. Returns true if distance is within switch range
                if not self.phases[0].end:
                    # Removes priority to prevent car from steering to the side
                    temp = self.phases[0].priority[0]
                    self.phases[0].priority[0] = self.phases[0].priority[1]
                    self.phases[0].priority[1] = temp
                    self.phases[0].end = True

            # Checks if no contour exists to switch mode (checks for consecutive no contour)
            if self.phases[0].end and self.phases[0].consec_no_contour >= Phase0.Const.MAX_CONSEC_NO_CONTOUR.value and self.min_dist_lane_ar <= 100:
                self.cur_phase = self.Phase.LANE_FOLLOW

        elif self.cur_phase == self.Phase.LANE_FOLLOW:
            # Searches for AR code with certain ID to detect when to switch to cone slaloming
            corners, ids = get_ar_markers(self.image)
            
            if ids is not None: 
                ids = ids[:, 0]

                # Checks for a tag with ID 2
                tag = [x for x in enumerate(ids) if x[1] == self.Const.SLALOM_AR_ID.value]
                if len(tag):
                    tag = tag[0]
                    corner = corners[tag[0]]

                    # Gets distance to AR tag (Used for state management)
                    pixel_coord = get_ar_center(corner)
                    pixel_coord = (int(pixel_coord[0]), int(pixel_coord[1]))
                    dist = rc_utils.get_pixel_average_distance(self.depth_image, pixel_coord)

                    if dist <= self.Const.SLALOM_AR_MIN_DIST.value:
                        self.cur_phase = self.Phase.SLALOM
                

    # Call this to run automatic slaloms
    def auto_control(self):
        self.update_images()

        self.phase_management()

        self.phases[self.cur_phase.value].run_main()

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

        self.update_images()

    # Main overhead function. Runs state machine
    def run_main(self):
        
        # Calls respective functions depending on if manual control was enabled
        if self.__is_manual:
            self.user_control()
        else:
            self.auto_control()






"""
.______    __    __       ___           _______. _______     __  
|   _  \  |  |  |  |     /   \         /       ||   ____|   /_ | 
|  |_)  | |  |__|  |    /  ^  \       |   (----`|  |__       | | 
|   ___/  |   __   |   /  /_\  \       \   \    |   __|      | | 
|  |      |  |  |  |  /  _____  \  .----)   |   |  |____     | | 
| _|      |__|  |__| /__/     \__\ |_______/    |_______|    |_|
"""



class Phase0:

    class State(IntEnum):
        pass

    class Const(Enum):
        BLUE = ((90, 120, 120), (120, 255, 255))
        RED = ((175, 120, 120), (5, 255, 255))
        GREEN = ((55, 120, 120), (80, 255, 255))

        MIN_CONTOUR_AREA = 30

        # Remap values from angle to speed (based on abs(self.angle))
        SPEED_REMAP = (1, 0, 0.3, 1)

        # How many frames to wait before confirming that the line ended (lane following entered)
        MAX_CONSEC_NO_CONTOUR = 3

        RAMP_ACCEL = -9.2

    def __init__(self):
        self.phase = 0

        self.angle = 0

        self.priority = []

        self.consec_no_contour = 0

        # Set to true by managemement when reached Lane AR code
        self.end = False

        # A crop window for the floor directly in front of the car
        self.CROP_FLOOR = ((260, 0), (rc.camera.get_height(), rc.camera.get_width()))

    def set_line_priority(self):
        # Gets visible AR tag corners and IDs
        corners, _ = get_ar_markers(TRIALS.image)

        # Gets the AR tag on the left based on direction
        left_ar = [corner for corner in corners if get_ar_direction(corner) == Direction.LEFT][0]

        # Tests each color range to find what color that pixel is
        colors = (self.Const.BLUE, self.Const.RED, self.Const.GREEN)

        # Adds that color as the first color in the priority map
        self.priority.append(get_ar_color(left_ar, TRIALS.hsv, colors))

        # Gets color at bottom center of image (the line that will need to be followed at the end)
        self.priority.append(colors[check_pixel_color(TRIALS.hsv[rc.camera.get_height() - 1, rc.camera.get_width() // 2], colors)])

        # DEBUG IMAGE DISPLAY
        '''img = ar.draw_ar_markers(TRIALS.image, corners, ids)
        #rc_utils.draw_circle(img, (int(avg_y), int(avg_x)), radius=2)
        rc_utils.draw_circle(img, target_pixel_index, radius=2)
        rc_utils.draw_circle(img, [rc.camera.get_height() - 1, rc.camera.get_width() // 2], radius=3)
        rc.display.show_color_image(img)'''    

    def run_main(self):
        image = np.copy(TRIALS.image) 
        image = rc_utils.crop(image, self.CROP_FLOOR[0], self.CROP_FLOOR[1])

        # Finds contour/line based on priority order
        found = False
        for col in self.priority:
            contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, col.value[0], col.value[1]), self.Const.MIN_CONTOUR_AREA.value)
            if contour is not None:
                found = True
                break
        if not found:
            contour = None
            self.consec_no_contour += 1
        else:
            self.consec_no_contour = 0

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

            rc.display.show_color_image(image)

            # Follow line
            kP = 2
            self.angle = rc_utils.clamp(rc_utils.remap_range(contour_center[1], 
                                                        rc.camera.get_width(), 
                                                        rc.camera.get_width() / 2 - 1, 1, 0) * kP, -1, 1)
        
        # Speed dampening with higher speeds
        speed = rc_utils.remap_range(abs(self.angle), *self.Const.SPEED_REMAP.value)

        y_accel = rc.physics.get_linear_acceleration()[1] 

        if y_accel > self.Const.RAMP_ACCEL.value:
            speed = -0.5

        rc.drive.set_speed_angle(speed, self.angle)


"""
.______    __    __       ___           _______. _______     ___         
|   _  \  |  |  |  |     /   \         /       ||   ____|   |__ \        
|  |_)  | |  |__|  |    /  ^  \       |   (----`|  |__         ) |       
|   ___/  |   __   |   /  /_\  \       \   \    |   __|       / /        
|  |      |  |  |  |  /  _____  \  .----)   |   |  |____     / /_        
| _|      |__|  |__| /__/     \__\ |_______/    |_______|   |____|       
                                                                         
"""


class Phase1:

    class State(IntEnum):
        HARD_STOP = 0
        SLOW = 1
        FAST = 2

    class Const(Enum):
        # When line following, how far away from the lane AR tag should priority be changed to prevent a crash
        PRIORITY_SWITCH_DIST = 415

        # When to do a sharp turn at the intersection
        TURN_DECISION_DIST = 157

        # When in fast state, dist from the slow contour needed to switch to slow state
        STATE_SWITCH_DIST = 50

        ORANGE = ((12, 100, 100), (21, 255, 255))
        PURPLE = ((130, 120, 120), (142, 255, 255))

        MIN_CONTOUR_AREA = 100

        ALIGN_REMAP = (-200, 200, -1, 1)
        SLOWDOWN_REMAP = (250, 30, 0.5, 0.1)

        DIR_NONE = 0
        DIR_RIGHT = 1
        DIR_LEFT = -1

        TAG_ID_COLOR = 1
        TAG_ID_TURN = 199

        SPEED_SLOW = 0.25

    def __init__(self):
        self.phase = 1

        self.fast_col = None #self.Const.ORANGE #None
        self.slow_col = None #self.Const.PURPLE #None

        self.cur_state = self.State.FAST

        # Used to save angle between loop calls
        self.angle = 0

        self.slow_state_angle = 0

        self.cropped_img = None

        self.stop_counter = 0

    def set_colors(self):
        # Searches for AR code with certain ID to detect when lane following is to be done 
        corners, ids = get_ar_markers(TRIALS.image)
        
        # Remove Priority? Crop Change?
        remove_priority = False
        change_crop = False

        dist = None

        if ids is not None: 
            ids = ids[:, 0]
            
            # Checks for a tag with ID 1
            tag = [x for x in enumerate(ids) if x[1] == self.Const.TAG_ID_COLOR.value]
            if len(tag):
                tag = tag[0]
                corner = corners[tag[0]]

                # Gets distance to AR tag (Used for state management)
                pixel_coord = get_ar_center(corner)
                pixel_coord = (int(pixel_coord[0]), int(pixel_coord[1]))
                dist = rc_utils.get_pixel_average_distance(TRIALS.depth_image, pixel_coord)

                # Looks for color and sets it if it hasn't already
                if self.fast_col is None:
                    # Finds color of tag and sets fast_col and slow_col
                    colors = (self.Const.ORANGE, self.Const.PURPLE)

                    output = get_ar_color(corner, TRIALS.hsv, colors, factor=1.7)
                    if output is not None:
                        self.fast_col = get_ar_color(corner, TRIALS.hsv, colors, factor=1.7)
                        self.slow_col = self.Const.ORANGE if self.fast_col == self.Const.PURPLE else self.Const.PURPLE

                if dist <= self.Const.PRIORITY_SWITCH_DIST.value:
                    remove_priority = True
                change_crop = True
        return remove_priority, change_crop, dist

    # Checks if Lane Follower AR tag is close enough and returns a constant
    def check_ar_turning(self) -> int:
        # Searches for AR code with certain ID to detect when lane following is to be done 
        corners, ids = get_ar_markers(TRIALS.image)

        # Finds corner with id corresponding to the turn tag ID
        corners = [corner for idx, corner in enumerate(corners) if ids[idx][0] == self.Const.TAG_ID_TURN.value]

        # If something was found...
        if len(corners):
            corner = corners[0]

            # Gets distance to AR tag
            pixel_coord = get_ar_center(corner)
            pixel_coord = (int(pixel_coord[0]), int(pixel_coord[1]))
            dist = rc_utils.get_pixel_average_distance(TRIALS.depth_image, pixel_coord)

            direction = get_ar_direction(corner)

            # If the car is close enough to the tag, decide the turn based on tag
            if dist <= self.Const.TURN_DECISION_DIST.value:
                return self.Const.DIR_LEFT if direction == Direction.LEFT else self.Const.DIR_RIGHT

        return self.Const.DIR_NONE

    def get_2_largest(self, img, col):
        # Gets all fast contours and sorts them by contour area
        contours = list(enumerate(rc_utils.find_contours(img, col.value[0], col.value[1])))
        areas = [rc_utils.get_contour_area(cont[1]) for cont in contours]
        contours = sorted(contours, key=lambda x: areas[x[0]], reverse=True)
        # Gets 2 largest contours
        largest = contours[:2]
        largest = [cont for cont in largest if areas[cont[0]] > self.Const.MIN_CONTOUR_AREA.value]

        return largest

    def run_slow(self):
        largest = self.get_2_largest(self.cropped_img, self.slow_col)

        if self.cur_state == self.State.SLOW:
            speed = self.Const.SPEED_SLOW.value
        else:
            speed = -1


        # If the fast lane is visible more than the slow one, then just keep turning in the previously set direction
        largest_fast_cropped = rc_utils.get_largest_contour(rc_utils.find_contours(self.cropped_img, self.fast_col.value[0],
                                                                                                self.fast_col.value[1]))
        largest_slow_cropped = rc_utils.get_largest_contour(rc_utils.find_contours(self.cropped_img, self.slow_col.value[0],
                                                                                                self.slow_col.value[1]))
            
        slow_a = rc_utils.get_contour_area(largest_slow_cropped) if largest_slow_cropped is not None else 0
        fast_a = rc_utils.get_contour_area(largest_fast_cropped) if largest_fast_cropped is not None else 0

        if len(largest) == 2 and not slow_a < fast_a:
            self.angle = 0

        elif len(largest) == 1 and not slow_a < fast_a:
            cont = largest[0][1]

            # Finds the top point of the contour and the bottom (Estimates line slope)
            top_pt = tuple([pt[0] for pt in cont if pt[0][1] == np.amin(cont[:, :, 1])][0])
            bott_pt = tuple([pt[0] for pt in cont if pt[0][1] == np.amax(cont[:, :, 1])][0])
            
            # Slop is sloppy?????????????
            if self.slow_state_angle == 0:
                if top_pt[0] - bott_pt[0] > 0:
                    self.slow_state_angle = 1
                elif top_pt[0] - bott_pt[0] < 0:
                    self.slow_state_angle = -1
                else:
                    self.slow_state_angle = 0

            self.angle = self.slow_state_angle

            # Draws VIZHUALS
            cv.line(self.cropped_img, top_pt, bott_pt, (255, 255, 0), thickness=2)

        rc.display.show_color_image(self.cropped_img)

        # Sets speed and angle
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(self.angle, -1, 1))

        return len(largest)

    def run_fast(self, dist_slow):
        # Get 2 largest fast contours
        largest = self.get_2_largest(self.cropped_img, self.fast_col)

        # Sets speed based on distance from sharp turn
        speed = rc_utils.remap_range(dist_slow, *self.Const.SLOWDOWN_REMAP.value)

        if len(largest) == 2:
            # Finds which contour is which (left side/ right side)
            avg_1 = np.average(largest[0][1][:, :, 0]) 
            avg_2 = np.average(largest[1][1][:, :, 0]) 

            avg_mid = (avg_2 + avg_1) / 2

            if avg_2 > avg_1:
                right_cont = largest[1][1]
                left_cont = largest[0][1]

            else:
                right_cont = largest[0][1]
                left_cont = largest[1][1]

            # Sets angle based on mid average offset from center
            self.angle = rc_utils.remap_range(avg_mid - (rc.camera.get_width() / 2), *self.Const.ALIGN_REMAP.value)

        # <>>>>>>  VISZHUALS
            rc_utils.draw_contour(self.cropped_img, left_cont, color=(255, 0, 0))
            rc_utils.draw_contour(self.cropped_img, right_cont, color=(0, 255, 0))

            cv.line(self.cropped_img, (int(avg_mid), 0), (int(avg_mid), rc.camera.get_height()), (255, 255, 0))

        # Sets speed and angle
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(self.angle, -1, 1))

        rc.display.show_color_image(self.cropped_img)


    def run_main(self):
        # print("FAST", self.fast_col, "SLOW", self.slow_col)
        self.cropped_img = np.copy(TRIALS.image)[rc.camera.get_height() * 2 // 3 : rc.camera.get_height(), :]

        direction = self.check_ar_turning()
        if not direction == self.Const.DIR_NONE:
            rc.drive.set_speed_angle(1, direction.value)

        else:
            # SLOW CONTOUR INFO GATHERING
            # Finds distance to largest slow contour >>
            largest_slow = rc_utils.get_largest_contour(rc_utils.find_contours(TRIALS.image, self.slow_col.value[0], self.slow_col.value[1]),
                                                        self.Const.MIN_CONTOUR_AREA.value)
            if largest_slow is not None:
                center_slow = rc_utils.get_contour_center(largest_slow)
                dist_slow = rc_utils.get_pixel_average_distance(TRIALS.depth_image, center_slow)
            else:
                dist_slow = 9999
            # -------------------------------------- <<

            if self.cur_state == self.State.FAST:
                self.run_fast(dist_slow)

                # If the the slow contour is within a certain range, switch states
                if dist_slow <= self.Const.STATE_SWITCH_DIST.value:
                    self.cur_state = self.State.HARD_STOP

            elif self.cur_state == self.State.SLOW or self.cur_state == self.State.HARD_STOP:
                if self.cur_state == self.State.HARD_STOP:
                    self.stop_counter += 1
                    if self.stop_counter >= 10:
                        self.stop_counter = 0
                        self.cur_state = self.State.SLOW

                # Runs function and gets output (# of slow contours visible)
                out = self.run_slow()
                if out == 0:
                    self.cur_state = self.State.FAST
                    self.slow_state_angle = 0
        
        
        print(self.cur_state) 
            
        # If slow line area sum is big enough, align to right side of fast lane:
        
        # If no visible fast lane:
        # ------ Full turn /or/ Consider way to turn on purple line (sharp turn)

        # If only one line visible:
        # ------ Save history of left side and right side contours and determine what side the single contour is on

        """rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        speed = rt - lt
        angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(angle, -1, 1))"""


"""
.______    __    __       ___           _______. _______     ____   
|   _  \  |  |  |  |     /   \         /       ||   ____|   |___ \  
|  |_)  | |  |__|  |    /  ^  \       |   (----`|  |__        __) | 
|   ___/  |   __   |   /  /_\  \       \   \    |   __|      |__ <  
|  |      |  |  |  |  /  _____  \  .----)   |   |  |____     ___) | 
| _|      |__|  |__| /__/     \__\ |_______/    |_______|   |____/  
                                                                   
"""
# Phase 2 last second slapped-together version in hopes of finishing the entire course....
class Phase2:

    class State(IntEnum):
        pass

    class Const(Enum):
        BLUE = ((90, 120, 120), (120, 255, 255))
        RED = ((130, 50, 50), (179, 255, 255))

    def __init__(self):
        self.phase = 3

    def run_main(self):
        contours = rc_utils.find_contours(TRIALS.image, self.Const.BLUE.value[0], self.Const.BLUE.value[1]) + rc_utils.find_contours(TRIALS.image, self.Const.RED.value[0], self.Const.RED.value[1])
     
        all_the_centers = [rc_utils.get_contour_center(c) for c in contours]
        all_the_centers = [cent for cent in all_the_centers if cent is not None]

        all_the_centers = np.array(all_the_centers)

        if len(all_the_centers):
            avg = np.average(all_the_centers[:, 1])
        else:
            avg = 0

        angle = rc_utils.remap_range(avg, 0, rc.camera.get_width(), -1, 1)

        rc.drive.set_speed_angle(1, angle)

        img = np.copy(TRIALS.image)
        [rc_utils.draw_contour(img, contour) for contour in contours]
        [rc_utils.draw_circle(img, (300, int(cent[1])), radius=3) for cent in all_the_centers]
        rc_utils.draw_circle(img, (300, int(avg)), radius=10)
        rc.display.show_color_image(img)

class Phase2_Unfinished_But_Effortful_version:

    class State(IntEnum):
        # Align with cone and move towards it while it is visible
        APPROACH = 0
        # Once cone is no longer visible (i.e no visible contours or next contour visible is not same color)
        # Car uses previous estimates of speed to pass the cone 
        PASS = 1
        # Depending on color, turn car until contour is found. If found, go to state 0
        TURN = 2

        # One time state to ensure the car enters the grassy area
        ENTRY = 3

    class Color(IntEnum):
        # Pass on the left
        BLUE = 0
        # Pass on the right
        RED = 1

    class Const(Enum):
        # Maximum distance change between frames to count as valid
        MAX_DIST_DIFF = 30

        # When passing visible cone, used as lower and higher boundaries for a proportion used to determine 
        # how far off-center of the screen to set the setpoint to not crash into the cone as the car gets close
        LOW_SET_PROPORTION = (90 / 320)
        HIGH_SET_PROPORTION = (320 / 320)
        # The distance boundaries used for the above proportion
        APPROACH_DIST_UPPER = 200
        APPROACH_DIST_LOWER = 27

        # How many cm to drive to pass a cone that went out of view
        PASS_DIST = 15
        PASS_TIME = 0.25

        # Consecutive no contours to start turn
        MAX_CONSEC_CONTOURS = 1

        KERNEL_SIZE = 5
        MIN_CONTOUR_AREA = 70
        MAX_CONTOUR_DIST = 300

        GRASS_ENTRY_DIST = 60

        CONE_SEARCH_WINDOW = (-50, 50)

    def __init__(self):
        self.phase = 2

        self.COLORS = {"BLUE": ((90, 120, 120), (120, 255, 255)), 
                       "RED": ((130, 50, 50), (179, 255, 255))}

        self.__cur_col = self.Color.BLUE
        self.__cur_state = self.State.ENTRY

        self.__depth_image = None
        self.__color_image = None
        self.__lidar = None
        self.update_images()

        # Variables for contour tracking
        self.__contour_distance = None
        self.__contour_center = None
        self.__contour = None
        self.update_contours()

        # Previous distance to cone
        self.__prev_distance = None
        # Used to calculate speed
        self.__prev_time = None

        # Consecutive frames where no contour was seen
        self.__consec_no_cont = 0

        self.__approach_angle = 0
        self.__approach_speed = 0
        self.__approach_setpoint = 0


    def update_contours(self):
        contours = rc_utils.find_contours(self.__color_image, self.COLORS[self.__cur_col.name][0], self.COLORS[self.__cur_col.name][1])

        out = Phase2.get_closest_contour(contours, self.__depth_image)

        if out is not None:
            # Saves previous measured distance
            if self.__contour_distance is not None:
                self.__prev_distance = self.__contour_distance

            self.__contour_distance, self.__contour, self.__contour_center = out

            # Draws closest contour
            rc_utils.draw_contour(self.__color_image, self.__contour)
            rc_utils.draw_circle(self.__color_image, self.__contour_center)
        else:
            self.__contour_distance = None
            self.__contour_center = None
            self.__contour = None
        
        # Displays image
        rc.display.show_color_image(self.__color_image)

    def update_images(self):
        self.__color_image = rc.camera.get_color_image()
        self.__depth_image = cv.GaussianBlur((rc.camera.get_depth_image() - 0.01) % 10000, (self.Const.KERNEL_SIZE.value, 
                                                                                            self.Const.KERNEL_SIZE.value), 0) 
        self.__lidar = (rc.lidar.get_samples() - 0.1) * 1000

    def get_closest_opposite_contour(self):
        # Finds closest contour of opposite color
        opp_col = self.Color.RED if self.__cur_col == self.Color.BLUE else self.Color.BLUE
        opp_conts = rc_utils.find_contours(self.__color_image, self.COLORS[opp_col.name][0], self.COLORS[opp_col.name][1])
        out = Phase2.get_closest_contour(opp_conts, self.__depth_image)
        
        # Gets the distance to the closest opposite color cone
        if out is not None:
            dist_opp = out[0]
        else:
            # A really high number that will always be greater than the closest desired cone
            dist_opp = 99999
        return dist_opp

    def run_pass_state(self):
        closest_lidar_sample = rc_utils.get_lidar_closest_point(self.__lidar, (0, 360))
        if 60 < closest_lidar_sample[0] < 300:
            self.__is_stamped = False
            self.__cur_state = self.State.TURN

    def run_turn_state(self):
        # >>> State Switch Detection
        # --------------------

        dist_opp = self.get_closest_opposite_contour()

        if self.__contour_distance is not None and dist_opp is not None:
            closest_cone_depth = min(self.__contour_distance, dist_opp)
        else:
            closest_cone_depth = None
        closest_cone_lidar = rc_utils.get_lidar_closest_point(self.__lidar, self.Const.CONE_SEARCH_WINDOW.value)[1]

        if closest_cone_lidar is not None and closest_cone_depth is not None:
            print(closest_cone_lidar)

        # Must have a cone of the needed color visible
        # and checks if the needed cone is closer than the closest opposite color cone
        if self.__contour_distance is not None and self.__contour_distance < dist_opp:
            #self.__cur_state = self.State.APPROACH
            rc.drive.set_speed_angle(0, 1)

        # >>> Turn to find cone
        # ---------------------
        else:
            speed = 1
        
            # If blue cone needs to be found next, turn left. Otherwise right
            angle = -1 if self.__cur_col == self.Color.BLUE else 1

            # Turns until cone is found
            speed, angle = 0, 1
            rc.drive.set_speed_angle(speed, angle)

    def run_entry_state(self):
        if self.__contour_distance is not None and self.__contour_distance <= self.Const.GRASS_ENTRY_DIST.value:
            self.__cur_state = self.State.APPROACH
        else:
            if self.__contour_center is None:
                angle = 1
            else:
                angle = rc_utils.remap_range(self.__contour_center[1], 0, rc.camera.get_width(), -1, 1)
            speed = 1
            rc.drive.set_speed_angle(speed, angle)

    def run_approach_state(self):
        # Calculates distance change to see if cone was passed
        if self.__contour_distance is not None:
            distance_change = self.__contour_distance - self.__prev_distance
            self.__consec_no_cont = 0
        else:
            distance_change = None
            self.__consec_no_cont += 1

        # If distance makes a huge jump, assume that it is now following a different cone so switch states
        if self.__consec_no_cont >= self.Const.MAX_CONSEC_CONTOURS.value or distance_change > self.Const.MAX_DIST_DIFF.value:
            self.__cur_col = self.Color.BLUE if self.__cur_col == self.Color.RED else self.Color.RED
            self.__cur_state = self.State.PASS

        else:
            # >>>> Proportional control
            # -------------------------

            # Scales the center offset boundaries based on screen width
            w = rc.camera.get_width()

            low_set = self.Const.LOW_SET_PROPORTION.value * (w / 2)
            high_set = self.Const.HIGH_SET_PROPORTION.value * (w / 2)

            # Calculates new setpoint based on how close the car is to the cone.
            if self.__contour_distance is not None:
                self.__approach_setpoint = rc_utils.remap_range(self.__contour_distance, self.Const.APPROACH_DIST_UPPER.value, 
                                                self.Const.APPROACH_DIST_LOWER.value, low_set, high_set) 
                                            
            # negates setpoint if color is red. This ensures that the car passes cone on correct side
            self.__approach_setpoint *= (-1 if self.__cur_col == self.Color.RED else 1)

            """DEBUG: Draw Setpoint"""
            #rc_utils.draw_circle(self.__color_image, (rc.camera.get_height() // 2, int((w / 2) + setpoint)))

            # Gets angle from setpoint using proportional control
            kP = 4
            if self.__contour_distance is not None:
                self.__approach_angle = rc_utils.clamp(rc_utils.remap_range(self.__contour_center[1], self.__approach_setpoint, 
                                                       rc.camera.get_width() + self.__approach_setpoint, -1, 1) * kP, -1, 1)
            
            #self.__approach_speed = rc_utils.remap_range(abs(self.__approach_angle), 1, 0, 0.1, 1)
            self.__approach_speed = 1

            # Sets speed angle
            rc.drive.set_speed_angle(self.__approach_speed, self.__approach_angle)

            """DEBUG: Show image"""
            #rc.display.show_color_image(self.__color_image)

    # Call this to run automatic slaloms
    def auto_control(self):
        if self.__cur_state == self.State.APPROACH:
            self.run_approach_state()
        elif self.__cur_state == self.State.PASS:
            self.run_pass_state()
        elif self.__cur_state == self.State.TURN:
            self.run_turn_state()
        elif self.__cur_state == self.State.ENTRY:
            self.run_entry_state()
        else:
            rc.drive.set_speed_angle(0, 0)

    # Main overhead function. Runs state machine
    def run_main(self):
        self.update_images()
        self.update_contours()

        self.auto_control()

        print("Distance", self.__contour_distance, "State", self.__cur_state.name)

    @staticmethod
    def get_closest_contour(contours, depth_img):
        # Gets centers of each contour and extracts countours based on conditions
        centers = []
        for idx, contour in enumerate(contours):
            cent = rc_utils.get_contour_center(contour)
            if cent is not None:
                dist = rc_utils.get_pixel_average_distance(depth_img, cent)
                area = rc_utils.get_contour_area(contour)
                if area > Phase2.Const.MIN_CONTOUR_AREA.value and dist < Phase2.Const.MAX_CONTOUR_DIST.value:
                    centers.append((idx, rc_utils.get_contour_center(contour)))

        indexes = [center[0] for center in centers]
        centers = [center[1] for center in centers]
        # Calculates the distance to each center
        distances = [rc_utils.get_pixel_average_distance(depth_img, (center[0], center[1])) for center in centers]

        conts = [contours[index] for index in indexes]

        # Finds smallest distance and index of that distance
        if len(conts):
            minimum = min(enumerate(distances), key=lambda x: x[1])
            # (index, min dist)

            # Returns distance to closest contour center, the contour itself, and the center position
            return (minimum[1], conts[minimum[0]], centers[minimum[0]])
        else:
            # If there is no contour, my love for humanities is returned
            return None

"""
.______    __    __       ___           _______. _______     _  _    
|   _  \  |  |  |  |     /   \         /       ||   ____|   | || |   
|  |_)  | |  |__|  |    /  ^  \       |   (----`|  |__      | || |_  
|   ___/  |   __   |   /  /_\  \       \   \    |   __|     |__   _| 
|  |      |  |  |  |  /  _____  \  .----)   |   |  |____       | |   
| _|      |__|  |__| /__/     \__\ |_______/    |_______|      |_|   
                                                                    
"""



class Phase3:

    class State(IntEnum):
        PANIC = 0
        STUFF = 1

    class Const(Enum):
        PANIC = 40

    def __init__(self):

        self.__right_wall = None
        self.__left_wall = None

        self.__depth = None

        self.cur_state = self.State.STUFF

        self.counter = 0

    def update_lidar(self):
        ang = rc_utils.get_lidar_closest_point(rc.lidar.get_samples(), (40, 50))[0]
        self.__right_wall = rc_utils.get_lidar_average_distance((rc.lidar.get_samples() - 0.1) % 1000, ang)

        ang =rc_utils.get_lidar_closest_point(rc.lidar.get_samples(), (-50, -40))[0]
        self.__left_wall = rc_utils.get_lidar_average_distance((rc.lidar.get_samples() - 0.1) % 1000, ang)

        self.__depth = rc.camera.get_depth_image()

    # Call this to run automatic slaloms
    def auto_control(self):
        #rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        #lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        #speed = rt - lt
        speed = 0.2
        angle = 0

        if self.cur_state == self.State.STUFF:
            if rc_utils.get_depth_image_center_distance(self.__depth) < self.Const.PANIC.value:
                self.cur_state = self.State.PANIC
                self.counter = 5
            else:

                diff = self.__right_wall - self.__left_wall
                angle = rc_utils.clamp(diff / 50, -1, 1)

                print("diff", diff, "ang",angle)

        else:
            self.counter -= 1
            angle = -1
            if self.counter <= 0:
                self.cur_state = self.State.PANIC
            

        rc.drive.set_speed_angle(speed, angle)

    # Main overhead function. Runs state machine
    def run_main(self):
        self.update_lidar()
        self.auto_control()



def gg():
    print("  _______   _______ \n"
          " /  _____| /  _____| \n"
          "|  |  __  |  |  __  \n"
          "|  | |_ | |  | |_ | \n"
          "|  |__| | |  |__| | \n"
          " \______|  \______| \n"
                            )


TRIALS = None

def start():
    global TRIALS
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Normal Mode
    #rc.drive.set_max_speed(0.25)
    # User Mode
    rc.drive.set_max_speed(0.7)
    # Fast Mode
    #rc.drive.set_max_speed(0.9)

    TRIALS = TimeTrials(manual_mode=False)

    # Print start message
    print(">> Final Challenge - Time Trials")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    
    TRIALS.run_main()























dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
params = cv.aruco.DetectorParameters_create()

class Direction(IntEnum):
    """
    AR marker direction
    """
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

def get_ar_markers(
    color_image: NDArray[(Any, Any, 3), np.uint8]
) -> Tuple[List[NDArray[(1, 4, 2), np.int32]], Optional[NDArray[(Any, 1), np.int32]]]:
    """
    Finds AR marker coordinates and ids in an image.

    """
    corners, ids, _ = cv.aruco.detectMarkers(
        color_image,
        dictionary,
        parameters=params
    )
    return (corners, ids)

def draw_ar_markers(
    color_image: NDArray[(Any, Any, 3), np.uint32],
    corners: List[NDArray[(1, 4, 2), np.int32]],
    ids: NDArray[(Any, 1), np.int32],
    color: Tuple[int, int, int] = (0, 255, 0),
) -> NDArray[(Any, Any, 3), np.uint8]:
    """
    Draw AR markers in a image, modifying original image.
    """
    return cv.aruco.drawDetectedMarkers(color_image, corners, ids, color)

def get_ar_direction(ar_corners):
    corns = ar_corners[0]
    for i in range(len(corns)):
        for j in range(len(corns[i])):
            corns[i][j] = (corns[i][j] // 10) * 10
    
    diff_x = (abs(np.amax(corns[:, 0]) - np.amin(corns[:, 0])) * 2) // 3
    diff_y = (abs(np.amax(corns[:, 1]) - np.amin(corns[:, 1])) * 2) // 3
    
    top_left = corns[0]
    bottom_left = corns[3]
    
    if top_left[0] - bottom_left[0] <= -diff_x:
        return Direction.LEFT
    elif top_left[1] - bottom_left[1] <= -diff_y:
        return Direction.UP
    elif top_left[0] - bottom_left[0] >= diff_x:
        return Direction.RIGHT
    else:
        return Direction.DOWN

def get_ar_center(corners):
    # Gets the center of the contour of the left AR tag
    x = corners[0, :, 0]
    y = corners[0, :, 1]
    avg_x = np.sum(x) / x.size
    avg_y = np.sum(y) / y.size

    return (avg_y, avg_x)

def get_ar_color(ar_tag, hsv, colors, factor=2):
    """ factor -> Indicates how far left to shift in the image to find the color outline around the AR tag box. The value
    is the number by which to divide half the width of the calculated AR bounding box"""
    # Gets the center of the contour of the left AR tag
    avg_y, avg_x = get_ar_center(ar_tag)

    # Finds the top left corner of the tag using the calculated center (the actual top left, not the orientation top left)
    actual_top_left = [corner for corner in ar_tag[0] if corner[0] < avg_x and corner[1] < avg_y][0]

    # Gets pixel a little to the left of the above top left coordinate (which should be the color around the AR tag box)
    target_pixel_index = int(actual_top_left[1]), int(actual_top_left[0] - ( ( avg_x - actual_top_left[0] ) / factor ))
    pixel = hsv[target_pixel_index]

    # Adds that color as the first color in the priority map
    idx = check_pixel_color(pixel, colors)
    if idx is None:
        return None
    else:
        return colors[idx]

def check_pixel_color(pixel, colors):
    for idx, col in enumerate([color.value for color in colors]):
        hue_match = False
        index = None

        # Checks if hue is within range. Accounts for ranges such as 170-15, which transition from end to beginning of hsv values
        if (col[0][0] > col[1][0] and (0 <= pixel[0] <= col[1][0] or col[0][0] <= pixel[0] <= 179)) or col[0][0] <= pixel[0] <= col[1][0]:
            hue_match = True
            
        if hue_match and col[0][1] <= pixel[1] <= col[1][1] and col[0][2] <= pixel[2] <= col[1][2]:
            index = idx
            break
    return index



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()