#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import math as m

def undistort(image):
    # initialize camera matrix [3x3] and distortion coefficients [1x5]
    mtx = np.array([[211.7015, 0, 184.2331],[0, 207.7593, 204.1617],[0, 0, 1]])
    dist = np.array([-0.0403, -0.1640, -0.0008, -0.0015, 0.1150])

    # optimize camera matrix
    h, w = image.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # undistort
    dst = cv.undistort(image, mtx, dist, None, newcameramtx)
    return dst

# get binary image of red dots, with smoothing function to erase single pixels
def getBinaryImage(image):
    undist_im = undistort(image)
    
    # Convert BGR to HSV
    hsv = cv.cvtColor(undist_im, cv.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red1 = np.array([0,1,140])
    upper_red1 = np.array([15,255,255])
    lower_red2 = np.array([170,1,140])
    upper_red2 = np.array([180,255,255])

 
    # Threshold the HSV image to get only red colors
    mask1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Average mask and threshold again to get rid of single stray pixels
    avg = cv.blur(mask, (4,4))
    ret,avgmask = cv.threshold(avg,127,255,cv.THRESH_BINARY)

    return avgmask

# Get connected components ("dot detection") and their coordinates
def getComponents(image):
    avgmask = getBinaryImage(image)
    components = cv.connectedComponentsWithStats(avgmask, 4, cv.CV_32S)
    return components

# get coordinates of red dots (and delete center of image as first coordinate)
def getCoordinates(image):
    components = getComponents(image)
    coordinates = components[3]
    
    # remove first center coordinates
    coordinates2 = np.delete(coordinates, 0,0)
    return coordinates2
    
# get intersection point between opposite points
def getCenterPoint(image):
    coordinates = getCoordinates(image)
    if len(coordinates) == 4:
        # points 1-4 and 2-3 are opposites
        c_1 = coordinates[0]
        c_2 = coordinates[1]
        c_3 = coordinates[2]
        c_4 = coordinates[3]

        # find intersection of 2 lines (found online)
        s = np.vstack([c_1,c_4,c_2,c_3])        # s for stacked
        h = np.hstack((s, np.ones((4, 1))))     # h for homogeneous
        l1 = np.cross(h[0], h[1])               # get first line
        l2 = np.cross(h[2], h[3])               # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            (float('inf'), float('inf'))
        center_coordinates = (x/z, y/z)
        return center_coordinates
    else:
        return 0
        
# get the area of the center blob in pixels
def getComponentArea(image):
    binImage = getBinaryImage(image)
    contours, hierarchy = cv.findContours(binImage, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    totalArea = 0
    for c in contours:
        area = cv.contourArea(c)
        totalArea = totalArea + area
    return totalArea
        
# get the average distance between opposite dots in pxl    
def getAvgPxlDistance(image):
    coordinates = getCoordinates(image)
    if len(coordinates) == 4:
        # points 1-4 and 2-3 are opposites
        c_1 = coordinates[0]
        c_2 = coordinates[1]
        c_3 = coordinates[2]
        c_4 = coordinates[3]

        # distance between opposites is hypotenuse
        dx1 = abs(c_4[0]-c_1[0])
        dy1 = abs(c_4[1]-c_1[1])
        d1 = m.sqrt(dx1**2 + dy1**2)
        dx2 = abs(c_3[0]-c_2[0])
        dy2 = abs(c_3[1]-c_2[1])
        d2 = m.sqrt(dx2**2 + dy2**2)
        avg_d = (d1 + d2) / 2

        return avg_d
    else:
        return False
    
# get the average distance between dots and center in pxl
def getAvgPxlDistanceToCenter(image):
    coordinates = getCoordinates(image)
    if len(coordinates) == 4:
        center = getCenterPoint(image)
        c_1 = coordinates[0]
        c_2 = coordinates[1]
        c_3 = coordinates[2]
        c_4 = coordinates[3]

        # distance between points and centerpoint
        dx1 = abs(c_1[0]-center[0])
        dy1 = abs(c_1[1]-center[1])
        d1 = m.sqrt(dx1**2 + dy1**2)
        dx2 = abs(c_2[0]-center[0])
        dy2 = abs(c_2[1]-center[1])
        d2 = m.sqrt(dx2**2 + dy2**2)
        dx3 = abs(c_3[0]-center[0])
        dy3 = abs(c_3[1]-center[1])
        d3 = m.sqrt(dx3**2 + dy3**2)
        dx4 = abs(c_4[0]-center[0])
        dy4 = abs(c_4[1]-center[1])
        d4 = m.sqrt(dx4**2 + dy4**2)
        avg_d = (d1 + d2 + d3 + d4) / 4

        return avg_d
    else:
        return 0
    
# get a numerical estimation for the distance from the the different dots    
def getDistance(image):
    d0 = 50          # focal distance of laser beams
    alpha = 10.2        # angle of lasers in degrees
    alpha_rad = alpha * m.pi / 180
    d_est = d0          # start the estimation at the focal point (where px_calc is 0)
    px_length = getAvgPxlDistanceToCenter(image)   # avg length from points to center in px
    px_calc = 0                             # calculated length from distance estimation
    px_diff = px_length - px_calc

    while px_diff <= -1 or px_diff >= 1:
        d_est = d_est + 1                   # increment distance estimation by 1 mm
        px_calc = 138.56 * d_est**-0.906 * (d_est-d0) * m.tan(alpha_rad)    # relation between px on image and distance between camera and wall
        px_diff = px_length - px_calc

    return d_est

# get the goal coordinates, color set to green for now
def getGoalCoordinates(image):
    # Convert BGR to HSV
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # define range of green color in HSV
    lower = np.array([36,25,25])
    upper = np.array([70,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv.inRange(hsv, lower, upper)

    # Average mask and threshold again to get rid of single stray pixels
    avg = cv.blur(mask, (4,4))
    ret,avgmask = cv.threshold(avg,127,255,cv.THRESH_BINARY)

    # Get goal component and its coordinates
    component = cv.connectedComponentsWithStats(avgmask, 4, cv.CV_32S)
    coordinates = component[3]
    # remove first center coordinates
    coordinates2 = np.delete(coordinates, 0,0)
    return coordinates2