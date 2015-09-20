#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
from njllrd_proj1.msg import * 
import baxter_interface
from std_msgs.msg import String

msg = state()

def handle_move_robot(req):
    action = req.action
    target = req.target
    # Returns true if action is valid and action is completed
    # Possible actions: open_gripper, close_gripper, move_to_block, move_over_block
    print "entered move_robot_callback"
    if action == 'open_gripper':
        if msg.gripper_state == 0 or msg.to_block == 1: 
            return False
        else:
            msg.gripper_state == 0
                # do something
            print "opened gripper!"
            return True
    elif action == 'close_gripper':
        if msg.gripper_state == 1 or msg.over_block == 1:
            return False
        else:
            msg.gripper_state == 1
            
            print "CLOSEd gripper!"
            return True
    elif action == 'move_to_block':
        if msg.gripper_state == 1 or msg.to_block == 1 or msg.over_block == 1:
            return False
        else:
            msg.to_block = 1
                # do something
            return True
    elif action == 'move_over_block':
        msg.over_block == 1
        print "moved over block %s" %(target)
        return True
    else:
        return False

    # Returns false if action is invalid or action fails
    # else return False

# Declares state of world at 1Hz using topic "state"
def broadcast_state():
    return True

def robot_interface():
    pub = rospy.Publisher("state",state,queue_size=10)
    rospy.init_node('robot_interface')
    rate = rospy.Rate(1)
    msg.gripper_state = 0 
    msg.to_block = 1
    msg.over_block = 0
    s = rospy.Service('move_robot', move_robot, handle_move_robot)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

    
    rospy.spin()

if __name__ == "__main__":
    robot_interface()
    




""" Move following two lines to whenever world state is initialized. 
    rospy.get_param('/global_num_blocks')
    rospy.get_param('/global_configuration') 
    Options: scattered, stacked_ascending, stacked_descending
"""
