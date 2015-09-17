#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
def handle_move_robot(action, target):
    
    # Returns true if action is valid and action is completed
    # Possible actions: open_gripper, close_gripper, move_to_block, move_over_block
    return True

    # Returns false if action is invalid or action fails
    # else return False

# Declares state of world at 1Hz using topic "state"
def broadcast_state():
    return True

def robot_interface():
    rospy.init_node('robot_interface')
    s = rospy.Service('move_robot', move_robot, handle_move_robot)
    print "Ready to accept requests"
    rospy.spin()

if __name__ == "__main__":
    robot_interface()
    




""" Move following two lines to whenever world state is initialized. 
    rospy.get_param('/global_num_blocks')
    rospy.get_param('/global_configuration') 
    Options: scattered, stacked_ascending, stacked_descending
"""
