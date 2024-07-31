#!/usr/bin/env python3
import numpy as np
from model import modelState
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

def model_callback(msg: Vector3):
    v = np.array([msg.x, msg.y, msg.z])
    fr = 30
    dv = v/fr

    model.move_x(dv[0])
    model.move_y(dv[1])
    model.move_z(dv[2])

if __name__ == "__main__":
    # initialize model
    distance = 300
    angle = 0
    goal = [112, -125]
    model = modelState(distance, angle, goal)

    rospy.init_node("model_node")
    pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)
    sub = rospy.Subscriber("/velocity", Vector3, callback=model_callback)
    rospy.loginfo("model node has been started")
    
    bridge = CvBridge()
    rate = rospy.Rate(30)

    # create first image to start working with
    image = model.createImage()
    im_msg = bridge.cv2_to_imgmsg(image, encoding="rgb8")
    pub.publish(im_msg)

    while not rospy.is_shutdown():
        image = model.createImage()
        im_msg = bridge.cv2_to_imgmsg(image, encoding="rgb8")
        pub.publish(im_msg)
        rate.sleep()