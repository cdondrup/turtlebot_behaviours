#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool


class PredatorSpotter(object):
    def __init__(self, name):
        rospy.loginfo("Creating " + name + " ...")
        self.simulator = rospy.get_param("~simulator", True)
        if self.simulator:
            rospy.loginfo("Just testing")
        else:
            rospy.loginfo("The games are over. This counts!")

        self.visualise = rospy.get_param("~visualise", True)
        if self.visualise:
            cv2.namedWindow("Image window", 1)
            cv2.startWindowThread()
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/predator_spotter/spotted", Bool, queue_size=10)
        rospy.Subscriber(
            "/turtlebot_2/camera/rgb/image_raw" \
                if self.simulator else \
                "/camera/rgb/image_color",
            Image,
            self.callback,
            queue_size=1
        )


    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e



        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if self.simulator:
            low = np.array((60, 120 , 0))
            up = np.array((110, 255, 255))
        else:
            low = np.array((51, 51 , 0))
            up = np.array((102, 127, 255))

        hsv_thresh = cv2.inRange(
            hsv_img,
            low,
            up
        )

        size = np.sum(hsv_thresh/255.0)
        #print size
        contours, hier = cv2.findContours(hsv_thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if 1000<cv2.contourArea(cnt):
                cv2.drawContours(cv_image,[cnt],0,(0,0,255),2)
                x,y,w,h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                #print aspect_ratio
                if aspect_ratio > 1.5 and cv2.contourArea(cnt) > 2000:
                    self.pub.publish(True)
                    print "RUN FORSET RUN!"

        if self.visualise:
            cv2.imshow("Image window", cv_image)


if __name__ == '__main__':
    rospy.init_node('predator_spotter')
    p = PredatorSpotter(rospy.get_name())
    rospy.spin()
