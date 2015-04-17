#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import yaml
from os import listdir
from os.path import isfile, join

from turtlebot_prey_behaviour.msg import PreyAction
from move_base_msgs.msg import MoveBaseAction
from turtlebot_prey_behaviour.choose_waypoint import ChooseWaypoint
from turtlebot_prey_behaviour.move_to_waypoint import MoveToWaypoint
from turtlebot_prey_behaviour.wait_at_waypoint import WaitAtWaypoint
from turtlebot_prey_behaviour.run_bitch_run import RunBitchRun


class StateMachine(object):
    def __init__(self, name):
        rospy.loginfo("Creating " + name + " server...")
        self._as = actionlib.SimpleActionServer(
            name,
            PreyAction,
            self.execute,
            auto_start=False
        )
        self._as.register_preempt_callback(self.preempt_callback)

        self.simulator = rospy.get_param("~simulator", True)
        if self.simulator:
            rospy.loginfo("Just testing")
        else:
            rospy.loginfo("The games are over. This counts!")

        rospy.loginfo("Waiting for move_base client ...")
        client = actionlib.SimpleActionClient("/turtlebot_2/move_base", MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo(" ... done")

        rospy.loginfo("Reading waypoints")
        self.waypoints = self.read_waypoints(rospy.get_param("~waypoint_dir"))

        rospy.loginfo(" ... starting " + name)
        self._as.start()
        rospy.loginfo(" ... started " + name)

    def read_waypoints(self, directory):
        waypoints = []
        yaml_files = [ join(directory,f) for f in listdir(directory) if isfile(join(directory,f))]
        for f in yaml_files:
             pose = yaml.load(open(f, 'r'))
             waypoints.append(pose["pose"])
        print waypoints
        return waypoints

    def execute(self, goal):
        rospy.loginfo("Starting state machine")

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        self.sm.userdata.waypoint = self.waypoints[0]
        sis = smach_ros.IntrospectionServer(
            'prey_state_machine',
            self.sm,
            '/prey_state_machine'
        )
        sis.start()
        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add(
                'CHOOSE_WAYPOINT',
                ChooseWaypoint(self.waypoints),
                transitions={
                    'done': 'MOVE_TO_WAYPOINT',
                    'killall': 'preempted'
                },
                remapping={'waypoint' : 'waypoint'}
            )
            smach.StateMachine.add(
                'MOVE_TO_WAYPOINT',
                MoveToWaypoint(self.simulator),
                transitions={
                    'reached': 'WAIT_AT_WAYPOINT',
                    'failed': 'CHOOSE_WAYPOINT',
                    'spotted_predator': 'RUN_BITCH_RUN',
                    'killall': 'preempted'
                },
                remapping={'waypoint' : 'waypoint'}
            )
            smach.StateMachine.add(
                'WAIT_AT_WAYPOINT',
                WaitAtWaypoint(self.simulator),
                transitions={
                    'done': 'CHOOSE_WAYPOINT',
                    'spotted_predator': 'RUN_BITCH_RUN',
                    'killall': 'preempted'
                },
                remapping={'waypoint' : 'waypoint'}
            )
            smach.StateMachine.add(
                'RUN_BITCH_RUN',
                RunBitchRun(self.simulator),
                transitions={
                    'done': 'CHOOSE_WAYPOINT',
                    'killall': 'preempted'
                },
                remapping={'waypoint' : 'waypoint'}
            )

        # Execute SMACH plan
        self.sm.execute()

        sis.stop()
        if not self._as.is_preempt_requested() and self._as.is_active():
            self._as.set_succeeded()
        else:
            self._as.set_preempted()

    def preempt_callback(self):
        rospy.logwarn("Preempt requested")
        self.sm.request_preempt()


if __name__ == '__main__':
    rospy.init_node('prey_state_machine')
    s = StateMachine(rospy.get_name())
    rospy.spin()
