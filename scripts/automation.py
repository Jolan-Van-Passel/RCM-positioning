import cv2 as cv
from model import modelState
from image_operations import *
from visual_servoing import *

# move camera based on visual servoing while it has 4 dots
def move_4dots(image):
    # get initial data for loop
    coordinates = getCoordinates(image)

    # movement loop
    # only works while there are 4 dots
    while len(coordinates) == 4:
        cv.imshow('image', image)

        # save previous pxl distance for safety check
        prev_dist = getAvgPxlDistance(image)

        # goal can disappear behind dots
        if len(getGoalCoordinates(image)) == 0:
            print('goal has disappeared')
            break
        
        # camera velocity in mm/s
        v = getCameraVelocity_4dots(image)

        # move camera by velocity
        dx = v[0]
        dy = v[1]
        dz = v[2]

        # get coordinates for next step
        coordinates = getCoordinates(image)

        # safety, if px distance between dots gets bigger => rcm is behind goal => stop before we crash into anything
        dist = getAvgPxlDistance(image)
        if prev_dist < dist:
            print('rcm went beyond goal')
            break

# move camera based on visual servoing while it has 1 dot
def move_1dot(image):
    # get initial image and data for loop
    area = getComponentArea(image)
    error = getError_1dot(image)

    # movement loop
    # only works while there is 1 dot
    while error[0] > 1 or error[1] > 1 or error[2] < -1:
        cv.imshow('image', image)

        # save previous area for safety check
        prev_area = getComponentArea(image)

        # goal can disappear behind dots
        if len(getGoalCoordinates(image)) == 0:
            print('goal has disappeared')
            break
        
        # camera velocity in mm/s
        v = getCameraVelocity_1dot(image)

        # move camera by velocity times 0.05s per frame
        dx = v[0]
        dy = v[1]
        dz = v[2]

        # get area and error for next step
        area = getComponentArea(image)
        error = getError_1dot(image)

        # safety, if area gets bigger => rcm is behind goal => stop before we crash into anything
        if prev_area < area:
            print('rcm went beyond goal')
            break
    print(error)