#!/usr/bin/env python

import rospy
import smach
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveToWaypoint(smach.State):
    def __init__(self, simulator):
        smach.State.__init__(
            self,
            outcomes=['reached', 'failed', 'spotted_predator', 'killall'],
            input_keys=['waypoint'],
            output_keys=['waypoint']
        )

        self.move_client = actionlib.SimpleActionClient("/turtlebot_2/move_base" if simulator else "/move_base", MoveBaseAction)
        self.move_client.wait_for_server()

        self.run_bitch = False
        rospy.Subscriber("/predator_spotter/spotted", Bool, self.callback)

    def execute(self, userdata):
        mbg = MoveBaseGoal()
        mbg.target_pose.header.frame_id = "map"
        mbg.target_pose.header.stamp = rospy.Time.now()
        mbg.target_pose.pose = userdata.waypoint
        self.move_client.send_goal_and_wait(mbg)
        state = self.move_client.get_state()

        if self.preempt_requested() and self.run_bitch:
            return 'spotted_predator'
        elif self.preempt_requested():
            return 'killall'
        else:
            if state == GoalStatus.SUCCEEDED:
                return 'reached'
            else:
                return 'failed'

    def request_preempt(self):
        """Overload the preempt request method."""
        self.move_client.cancel_all_goals()
        smach.State.request_preempt(self)

    def callback(self, data):
        if data.data:
            rospy.loginfo("Spotted predator!!")
            self.run_bitch = True
            self.request_preempt()

