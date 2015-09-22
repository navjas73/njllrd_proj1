#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
from njllrd_proj1.msg import *
import time
from std_msgs.msg import String

# When topic "command" is published to, reads block orientation mode and
#   starts relevant process
def mode_selection(data):
    if data.data == "scatter":
        scatter()
    elif data.data == "stack_ascending":
        stack_ascending()
    elif data.data == "stack_descending":
        stack_descending()
    elif data.data == "odd_even":
        odd_even()
    else: 
        print("Invalid mode")


# Initiates request to move_robot service (handled by robot interface server)
def request_movement(action, target):
    try:
        request = rospy.ServiceProxy('move_robot', move_robot)
        output = request(action,target)
        if output == False:
            print "Invalid movement"
        else:
            print "Successfully moved"
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e
    
def scatter():
    # Code for moving blocks with scattered orientation
    return

def stack_ascending():
    # Code for moving blocks with stacked ascending orientation
    request_movement("close_gripper", 1)
    request_movement("move_over_block", 0) # virtual block (table)
    # request_movement("open_gripper", 1)

def stack_descending():
    # Code for moving blocks with stacked descending orientation
    return

def odd_even():
    return

def read_state():
    return

def controller():
    rospy.init_node('controller')
    rospy.Subscriber('command', String, mode_selection)
    # rospy.Subscriber('state', state, read_state)
    rospy.wait_for_service('move_robot')
    # time.sleep(10)
    rospy.spin()

if __name__ == "__main__":
    controller()
    


