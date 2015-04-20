#!/usr/bin/env python

import rospy
import smach
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi
from random import choice


class RunBitchRun(smach.State):
    def __init__(self, simulator):
        smach.State.__init__(
            self,
            outcomes=['done', 'killall'],
            input_keys=['waypoint'],
            output_keys=['waypoint']
        )
        self.run_bitch = False
        self.min_range = 100.0
        self.pub = rospy.Publisher("/turtlebot_2/cmd_vel" if simulator else "/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/turtlebot_2/scan" if simulator else "/scan", LaserScan, self.callback)

    def execute(self, userdata):
        rospy.loginfo("DANGER WILL ROBINSON! DANGER!")
        end = rospy.Time.now().to_sec() + 3.0
        while end > rospy.Time.now().to_sec() and not self.run_bitch and not self.preempt_requested() and not rospy.is_shutdown():
            turn = Twist()
            turn.linear.x = 0.
            turn.angular.z = pi/2
            rospy.sleep(0.1)
            self.pub.publish(turn)

        while self.min_range > 1.0 and not self.run_bitch and not self.preempt_requested() and not rospy.is_shutdown():
            run = Twist()
            run.linear.x = 0.5
            run.angular.z = 0.0
            rospy.sleep(0.1)
            self.pub.publish(run)

        end = rospy.Time.now().to_sec() + 3.0
        speed = pi/4 * choice([-1,1])
        while end > rospy.Time.now().to_sec() and not self.run_bitch and not self.preempt_requested() and not rospy.is_shutdown():
            turn = Twist()
            turn.linear.x = 0.
            turn.angular.z = speed
            rospy.sleep(0.1)
            self.pub.publish(turn)

        if self.preempt_requested():
            return 'killall'
        else:
            return 'done'

    def callback(self, data):
        self.min_range = np.nanmin(data.ranges)
