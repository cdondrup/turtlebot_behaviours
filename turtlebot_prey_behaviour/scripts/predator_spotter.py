#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from dynamic_reconfigure.server import Server as DynServer
from turtlebot_prey_behaviour.cfg import SpotterConfig


class PredatorSpotter(object):
    def __init__(self, name):
        rospy.loginfo("Creating " + name + " ...")
        self.simulator = rospy.get_param("~simulator", True)
        if self.simulator:
            rospy.loginfo("Just testing")
        else:
            rospy.loginfo("The games are over. This counts!")

        self.dyn_srv = DynServer(SpotterConfig, self.dyn_callback)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/predator_spotter/spotted", Bool, queue_size=10)
        self.img_pub = rospy.Publisher("/predator_spotter/image", Image, queue_size=1)
        rospy.Subscriber(
            "/turtlebot_2/camera/rgb/image_raw" \
                if self.simulator else \
                "/camera/rgb/image_color",
            Image,
            self.callback,
            queue_size=1
        )

    def dyn_callback(self, config, level):
        self.hue_low       = config["hue_low"]
        self.hue_high       = config["hue_high"]
        self.value_low       = config["value_low"]
        self.value_high       = config["value_high"]
        return config


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
            low = np.array((self.hue_low, self.value_low , 0))
            up = np.array((self.hue_high, self.value_high, 255))

        hsv_thresh = cv2.inRange(
            hsv_img,
            low,
            up
        )

        kernel = np.ones((3,3),np.uint8)
        thresh = cv2.medianBlur(hsv_thresh,5)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        kernel = np.ones((10,10),np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_DILATE, kernel)
        #print size
        contours, hier = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if 1000<cv2.contourArea(cnt):
                cv2.drawContours(cv_image,[cnt],0,(0,0,255),2)
                x,y,w,h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                #print aspect_ratio
                if aspect_ratio > 1.5 and cv2.contourArea(cnt) > 2000:
                    self.pub.publish(True)
                    print "RUN FORSET RUN!"
        if self.img_pub.get_num_connections():
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))


if __name__ == '__main__':
    rospy.init_node('predator_spotter')
    p = PredatorSpotter(rospy.get_name())
    rospy.spin()
