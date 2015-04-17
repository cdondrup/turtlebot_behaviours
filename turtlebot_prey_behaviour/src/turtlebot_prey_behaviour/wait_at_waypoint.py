#!/usr/bin/env python

import rospy
import smach
from random import randint
from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import Bool


class WaitAtWaypoint(smach.State):
    def __init__(self, simulator):
        smach.State.__init__(
            self,
            outcomes=['done', 'spotted_predator', 'killall'],
            input_keys=['waypoint'],
            output_keys=['waypoint']
        )
        self.run_bitch = False
        self.pub = rospy.Publisher("/turtlebot_2/cmd_vel" if simulator else "/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/predator_spotter/spotted", Bool, self.callback)

    def execute(self, userdata):
        duration = randint(5,10)
        rospy.loginfo("Waiting at waypoint for %i seconds" % duration)
        end = rospy.Time.now().to_sec() + float(duration)
        while end > rospy.Time.now().to_sec() and not self.run_bitch and not self.preempt_requested() and not rospy.is_shutdown():
            turn = Twist()
            turn.linear.x = 0.
            turn.angular.z = pi/4
            rospy.sleep(0.1)
            self.pub.publish(turn)

        if self.preempt_requested() and self.run_bitch:
            return 'spotted_predator'
        elif self.preempt_requested():
            return 'killall'
        else:
            return 'done'

    def callback(self, data):
        if data.data:
            rospy.loginfo("Spotted predator!!")
            self.run_bitch = True
            self.request_preempt()
