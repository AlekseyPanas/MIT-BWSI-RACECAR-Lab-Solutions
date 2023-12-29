import cv2 as cv
import numpy as np
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import IntEnum

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