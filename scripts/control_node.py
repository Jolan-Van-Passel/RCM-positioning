#!/usr/bin/env python3
from image_operations import *
from visual_servoing import *
import rospy
import time
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def control_callback(msg: Image):
    global timeOfLastCallback
    timeOfLastCallback = time.time_ns()//1000000
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    coordinates = getCoordinates(image)
    global v
    
    if len(coordinates) == 4:
        v = getCameraVelocity_4dots(image)
    else:
        try:
            if -1 <= int(getError_1dot(image)[0]) <= 1 and -1 <= int(getError_1dot(image)[1]) <= 1 and -10 <= int(getError_1dot(image)[2]) <= 10:
                global goalReached
                goalReached = True
        except:
            rospy.loginfo("goal not found")
            v.x = 0
            v.y = 0
            v.z = 0
        v = getCameraVelocity_1dot(image)
    pub.publish(v)
    rospy.loginfo(str(v.x) + "\n" + str(v.y) + "\n" + str(v.z))

if __name__ == '__main__':
    rospy.init_node("control_node")
    pub = rospy.Publisher("/velocity", Vector3, queue_size=10)
    pub2 = rospy.Publisher("/masked_image", Image, queue_size=10)
    sub = rospy.Subscriber("/usb_cam/image_raw", Image, callback=control_callback)
    rospy.loginfo("control node has been started")

    rate = rospy.Rate(30)
    v = Vector3()
    timeOfLastCallback = time.time_ns()//1000000
    goalReached = False
    
    while not rospy.is_shutdown():
        currentTime = time.time_ns()//1000000
        if currentTime - timeOfLastCallback > 66:
            v.x = 0
            v.y = 0
            v.z = 0
            pub.publish(v)
            rospy.loginfo("velocity set to zero due to lack of input")
        if goalReached:
            v.x = 0
            v.y = 0
            v.z = 0
            rospy.loginfo("goal has been reached")
            pub.publish(v)
            rospy.signal_shutdown("goal has been reached")
        rate.sleep()
    