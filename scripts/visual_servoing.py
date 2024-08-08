import numpy as np
import math as m
from image_operations import *
from geometry_msgs.msg import Vector3
import rospy

def getInteractionMatrix_4dots(image):
    # angle of the cone
    alpha = 10.2
    r_alpha = alpha * m.pi / 180
    
    # get distance based on how far dots are apart in image
    d = getDistance(image)

    # px/mm ratio based on distance
    a = 138.56 * d**-0.906
    
    # interaction matrix
    L = np.asarray(
        [[a,0,0],
         [0,a,-a*m.tan(r_alpha)],
         [a,0,a*m.tan(r_alpha)],
         [0,a,0],
         [a,0,-a*m.tan(r_alpha)],
         [0,a,0],
         [a,0,0],
         [0,a,a*m.tan(r_alpha)]])
    return L

def getError_4dots(image):
    g = getGoalCoordinates(image)
    c = getCoordinates(image)

    # convert image coordinates to model coordinates
    if g.shape[0] > 0 and g.shape[1] > 0:
        goal = [g[0][0]-200, 200-g[0][1]]
        coordinates = [[c[0][0]-200, 200-c[0][1]],
                   [c[1][0]-200, 200-c[1][1]],
                   [c[2][0]-200, 200-c[2][1]],
                   [c[3][0]-200, 200-c[3][1]]]
    
        # error is difference between goal and coordinates
        e = np.asarray(
            [goal[0]-coordinates[0][0],
            goal[1]-coordinates[0][1],
            goal[0]-coordinates[1][0],
            goal[1]-coordinates[1][1],
            goal[0]-coordinates[2][0],
            goal[1]-coordinates[2][1],
            goal[0]-coordinates[3][0],
            goal[1]-coordinates[3][1]])
    else:
        e=0
    return e

def getCameraVelocity_4dots(image):
    theta = 0       # downwards angle between z axis of base and z_prime axis of camera
    
    L = getInteractionMatrix_4dots(image)
    e = getError_4dots(image)
    
    # factor to multiply speed in [1/s]
    K = 0.1
    
    v = Vector3
    try:
        if e == 0:
            v.x = 0
            v.y = 0
            v.z = 0
    except:
        # desired pixel speed of points
        u_d = K*e

        # Moore-Penrose pseudo-inverse interaction matrix
        L_inv = np.linalg.pinv(L)

        # camera speed is dot product of inverse L and desired pxl speed
        v_prime = np.dot(L_inv, u_d)

        # adaption for different coordinate system
        v.x = v_prime[0]
        v.y = v_prime[1] * m.cos(theta) - v_prime[2] * m.sin(theta)
        v.z = v_prime[1] * m.sin(theta) + v_prime[2] * m.cos(theta)
    return v

def getInteractionMatrix_1dot(image):
    d0 = 50
    
    # get area of points
    A = getComponentArea(image)

    # px/mm ratio based on distance
    a = 138.56 * d0**-0.906

    # interaction matrix
    L = np.asarray(
        [[a,0,0],
         [0,a,0],
         [0,0,-11.42]]) # -11.42 for model atm
    return L

def getError_1dot(image):
    g = getGoalCoordinates(image)
    #rospy.loginfo(g)
    c = getCoordinates(image)
    goal_A = 467 # 65 for model, 467 for irl atm
    area = getComponentArea(image)

    # convert image coordinates to model coordinates
    if g.shape[0] > 0 and g.shape[1] > 0 and c.shape[0] > 0 and c.shape[1] > 0:
        goal = [g[0][0]-200, 200-g[0][1]]
        coordinates = [[c[0][0]-200, 200-c[0][1]]]
        
        # error is difference between goal and coordinates
        e = np.asarray(
            [goal[0]-coordinates[0][0],
            goal[1]-coordinates[0][1],
            goal_A-area])
        #rospy.loginfo(e)
    else:
        e=0
    return e

def getCameraVelocity_1dot(image):
    theta = 0       # downwards angle between z axis of base and z_prime axis of camera
    
    L = getInteractionMatrix_1dot(image)
    e = getError_1dot(image)
    
    # factor to multiply speed in [1/s]
    K = 1

    v = Vector3 
    try:
        if e == 0:
            v.x = 0
            v.y = 0
            v.z = 0
    except:
        # desired pixel speed of points
        u_d = K*e

        # Moore-Penrose pseudo-inverse interaction matrix
        L_inv = np.linalg.pinv(L)

        # camera speed is dot product of inverse L and desired pxl speed
        v_prime = np.dot(L_inv, u_d)

        # adaption for different coordinate system
        v.x = v_prime[0]
        v.y = v_prime[1] * m.cos(theta) - v_prime[2] * m.sin(theta)
        v.z = v_prime[1] * m.sin(theta) + v_prime[2] * m.cos(theta)
    return v