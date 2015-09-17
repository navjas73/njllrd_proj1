#!/usr/bin/env python

import rospy
from njllrd_proj1.srv import *
import sys

def robot_interface_client(action,target):
    rospy.wait_for_service('move_robot')
    try:
        move_robot = rospy.ServiceProxy('move_robot',move_robot)
        success = move_robot(action,target)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    action = sys.argv[1]
    target = sys.argv[2]
    print "Success? %s" %robot_interface_client(action,target)

