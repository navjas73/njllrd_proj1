#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
from njllrd_proj1.msg import *
import time
from std_msgs.msg import String

# When topic "command" is published to, reads block orientation mode and
#   starts relevant process
def mode_selection(data):
    dual_arm = rospy.get_param("/dual_arm_mode")
    if dual_arm == False:
        if data.data == "scatter":
            scatter()
        elif data.data == "stack_ascending":
            if rospy.get_param("/configuration") != "stacked_ascending":
                stack_ascending()
            else:
                print("Already stacked ascending")
        elif data.data == "stack_descending":
            if rospy.get_param("/configuration") != "stacked_descending":                
                stack_descending()
            else:
                print("Already stacked descending")
        elif data.data == "odd_even":
            odd_even()
        else: 
            print("Invalid mode")
    else:
        if data.data == "scatter":
            dual_scatter()
        elif data.data == "stack_ascending":
            dual_stack_ascending()
        elif data.data == "stack_descending":
            dual_stack_descending()
        elif data.data == "odd_even":
            dual_odd_even()
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
    num_blocks = rospy.get_param("/num_blocks")
    rate = rospy.Rate(.5)
    for i in range(0,num_blocks):
        request_movement("close_gripper", 1)
        rate.sleep()
        print("move over block" + str(-i-1))
        request_movement("move_over_block", -i-1) # virtual block (table)
        rate.sleep()
        request_movement("open_gripper", 1)
        rate.sleep()
        if i != (num_blocks-1):
            print("move to block" + str(i+2))
            request_movement("move_to_block",i+2)
            rate.sleep()
    return

def stack_ascending():
    # Code for stack_ascending with stacked descending orientation
    num_blocks = rospy.get_param("/num_blocks")
    rate = rospy.Rate(.5)
    for i in range(0,num_blocks):
        request_movement("close_gripper", 1)
        rate.sleep()
        print("move over block" + str(i))
        request_movement("move_over_block", i) # virtual block (table)
        rate.sleep()
        request_movement("open_gripper", 1)
        rate.sleep()
        if i != (num_blocks-1):
            print("move to block" + str(i+2))
            request_movement("move_to_block",i+2)
            rate.sleep()
    # Code for stack_ascending with stacked ascending orientation
        

def stack_descending():
    # Code for stack_descending blocks with stacked ascending orientation




    # Code for stack_descending blocks with stacked descending orientation
    return

def odd_even():
    return

def dual_scatter():
    # Code for moving blocks with scattered orientation
    return

def dual_stack_ascending():
    # Code for stack_ascending with stacked descending orientation
    num_blocks = rospy.get_param("/num_blocks")
    rate = rospy.Rate(.5)
    for i in range(0,num_blocks):
        request_movement("close_gripper", 1)
        rate.sleep()
        print("move over block" + str(i))
        request_movement("move_over_block", i) # virtual block (table)
        rate.sleep()
        request_movement("open_gripper", 1)
        rate.sleep()
        if i != (num_blocks-1):
            print("move to block" + str(i+2))
            request_movement("move_to_block",i+2)
            rate.sleep()
    # Code for stack_ascending with stacked ascending orientation
        

def dual_stack_descending():
    # Code for stack_descending blocks with stacked ascending orientation




    # Code for stack_descending blocks with stacked descending orientation
    return

def dual_odd_even():
    return

def read_state():
    return

def controller():
    rospy.init_node('controller')
    rospy.wait_for_service('move_robot')
    rospy.Subscriber('command', String, mode_selection)
    # rospy.Subscriber('state', state, read_state)
    
    # time.sleep(10)
    rospy.spin()

if __name__ == "__main__":
    controller()
    


