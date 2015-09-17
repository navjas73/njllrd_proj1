#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
from njllrd_proj1.msg import *

# When topic "command" is published to, reads block orientation mode and
#   starts relevant process
def mode_selection(data):
    if data.data == "scatter"
        scatter()
    else if data.data = "stack_ascending"
        stack_ascending()
    else if data.data = "stack_descending"
        stack_descending()
    else if data.data = "odd_even"
        odd_even()
    else 
        print("Invalid mode")


# Initiates request to move_robot service (handled by robot interface server)
def request_movement(action, target):
    try:
        request = rospy.ServiceProxy('move_robot', move_robot)
        if request == False
            print "Invalid movement"
        else
            print "Successfully moved"
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e
    
def scatter():
    # Code for moving blocks with scattered orientation

def stack_ascending():
    # Code for moving blocks with stacked ascending orientation

def stack_descending():
    # Code for moving blocks with stacked descending orientation

def odd_even():

def read_state():

def controller():
    rospy.init_node('controller')
    rospy.Subscriber('command', String, mode_selection)
    rospy.Subscriber('state', state, read_state)
    rospy.spin()

if __name__ == "__main__":
    controller()
    


