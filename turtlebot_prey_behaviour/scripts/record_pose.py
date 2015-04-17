#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

import yaml
import time


saved = False

def callback(msg):
    print msg.pose.pose
    name = time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime(rospy.Time.now().to_sec())) + ".yaml"
    f = open(name, "w")
    yaml.dump({"pose": msg.pose.pose}, f, default_flow_style=False)
    f.close()
    global saved
    saved = True

if __name__ == '__main__':
    rospy.init_node('record_pose')

    directory = rospy.get_param("~dir")
    rospy.Subscriber(rospy.get_param("~topic","/amcl_pose"), PoseWithCovarianceStamped, callback)

    while not rospy.is_shutdown() and not saved:
        rospy.sleep(0.1)