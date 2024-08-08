#!/usr/bin/env python3
from image_operations import *
from visual_servoing import *
import rospy
import time
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge   

if __name__ == '__main__':
    rospy.init_node("control_node")
    pub = rospy.Publisher("/automation_velocity", Vector3, queue_size=20)
    pub2 = rospy.Publisher("/camera/image", Image, queue_size=20)
    cap = cv.VideoCapture(0, cv.CAP_V4L2)
    if not cap.isOpened():
        rospy.logerr("cannot open camera")
        rospy.signal_shutdown("cannot open camera")
    else:
        rospy.loginfo("control node has been started")

    rate = rospy.Rate(30)
    v = Vector3()
    goalReached = False
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        coordinates = getCoordinates(frame)

        if len(coordinates) == 4:
            v = getCameraVelocity_4dots(frame)
        else:
            rospy.logdebug("test1")
            try:
                if -1 <= int(getError_1dot(frame)[0]) <= 1 and -1 <= int(getError_1dot(frame)[1]) <= 1 and -10 <= int(getError_1dot(frame)[2]) <= 10:
                    goalReached = True
            except:
                rospy.loginfo("goal not found")
                v.x = 0
                v.y = 0
                v.z = 0
            v = getCameraVelocity_1dot(frame)
        if not ret:
            v.x = 0
            v.y = 0
            v.z = 0
            pub.publish(v)
            rospy.loginfo("velocity set to zero due to lack of input")
        elif goalReached:
            v.x = 0
            v.y = 0
            v.z = 0
            rospy.loginfo("goal has been reached")
            pub.publish(v)
            cap.release()
            cv.destroyAllWindows()
            rospy.signal_shutdown("control node has stopped")
        else:
            rospy.loginfo(str(v.x) + "\n" + str(v.y) + "\n" + str(v.z))
            pub.publish(v)
            image = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub2.publish(image_message)
        rate.sleep()
    
