import numpy as np
import math as m
import cv2 as cv

class modelState:
    def __init__(self, distance, angle, goal):
        self.d = distance           # distance to projection surface in mm
        self.beta = angle           # rotational angle of laser set up around y in degrees
        self.x = 0                  # translation of laser set up along x in mm
        self.y = 0                  # translation of laser set up along y in mm
        self.alpha = 10.2              # angle of cone in degrees
        self.b = 9                 # distance from center to laser center on cone surface in mm
        self.gx = goal[0]                 # x coordinate of goal on projection surface in mm
        self.gy = goal[1]                 # y coordinate of goal on projection surface in mm
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y

    def get_distance(self):
        return self.d

    # move along z by a certain distance in mm, sign defines direction
    def move_z(self, vector):
        self.d = self.d - vector

    # move along y by a certain distance in mm, sign defines direction
    def move_y(self, vector):
        self.y = self.y + vector

    # move along z by a certain distance in mm, sign defines direction
    def move_x(self, vector):
        self.x = self.x + vector

    def getGoal(self):
        goal = [self.gx, self.gy]
        return goal
    
    def setGoal(self, new_goal):
        self.gx = new_goal[0]
        self.gy = new_goal[1]

    def createImage(self):
        #calculations
        r_alpha = self.alpha*m.pi/180    # degrees to radians
        r_beta = self.beta*m.pi/180

        a = m.tan(r_alpha)          # angle coefficient of the lasers

        y1 = -a*self.d+self.b                 # intersection between laser and projection surface
        y2 = a*self.d-self.b
        x3 = a*self.d-self.b
        r_x3 = x3/(m.cos(r_beta)*(1+m.tan(r_beta)*m.tan(r_alpha)))      # corrected z value for rotated projection surface
        x4 = -a*self.d+self.b
        r_x4 = x4/(m.cos(r_beta)*(1+m.tan(r_beta)*m.tan(r_alpha)))

        # mm to px conversion
        ratio = 138.56 * self.d**-0.906
        y1_px = y1 * ratio
        y2_px = y2 * ratio
        x3_px = r_x3 * ratio
        x4_px = r_x4 * ratio

        #create image
        image = np.zeros((400,400,3), np.uint8)
        goal = cv.circle(image, (200-int(self.x)+self.gx,200+int(self.y)-self.gy), 10, (0,255,0), -1)
        p1 = cv.circle(image, (200,200+int(y1_px)), 5, (0,0,255), -1)
        p2 = cv.circle(image, (200,200+int(y2_px)), 5, (0,0,255), -1)
        p3 = cv.circle(image, (200+int(x3_px),200), 5, (0,0,255), -1)
        p4 = cv.circle(image, (200+int(x4_px),200), 5, (0,0,255), -1)
        return image