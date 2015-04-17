#!/usr/bin/env python

import rospy
import smach
from random import randint
from copy import deepcopy


class ChooseWaypoint(smach.State):
    def __init__(self, waypoints):
        smach.State.__init__(
            self,
            outcomes=['done', 'killall'],
            input_keys=['waypoint'],
            output_keys=['waypoint']
        )
        self.waypoints = deepcopy(waypoints)

    def execute(self, userdata):
        rospy.loginfo("Selecting waypoint")
        waypoints = deepcopy(self.waypoints)
        waypoints.remove(userdata.waypoint)
        userdata.waypoint = waypoints[randint(0, len(waypoints)-1)]

        if self.preempt_requested():
            return 'killall'
        else:
            return 'done'
