#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
from njllrd_proj1.msg import *
import time
from std_msgs.msg import String

working = False


# When topic "command" is published to, reads block orientation mode and
#   starts relevant process
def mode_selection(data):
    dual_arm = rospy.get_param("/dual_arm_mode")
    if working == True:
        print "Cannot accept commands while moving"
    else:
        if dual_arm == False:
            if data.data == "scatter":
                if rospy.get_param("/configuration") != "scattered":
                    scatter()
                else:
                    print("Already scattered")
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
    global working
    working = True
    rospy.set_param("/current_mode", "scatter")
    # Code for moving blocks with scattered orientation
    num_blocks = rospy.get_param("/num_blocks")
    rate = rospy.Rate(.5)
    for i in range(0,num_blocks):
        request_movement("close_gripper", -1)
        rate.sleep()
        print("move over block" + str(-i-1))
        request_movement("move_over_block", -i-1) # virtual block (table)
        rate.sleep()
        request_movement("open_gripper", -1)
        rate.sleep()
        if i != (num_blocks-1):
            if rospy.get_param("/configuration") == "stacked_descending":
                print("move to block" + str(i+2))
                request_movement("move_to_block",i+2)
            elif rospy.get_param("configuration") == "stacked_ascending":
                print("move to block" + str(num_blocks - i -1))
                request_movement("move_to_block",num_blocks - i - 1)
            rate.sleep()
    working = False
    print("Scatter Completed")
    rospy.set_param('/configuration',"scattered")
    return

def stack_ascending():
    global working
    working = True
    rospy.set_param("/current_mode", "stack_ascending")
    # Code for stack_ascending with stacked descending orientation
    num_blocks = rospy.get_param("/num_blocks")
    rate = rospy.Rate(.5)
    for i in range(0,num_blocks):
        request_movement("close_gripper", -1)
        rate.sleep()
        print("move over block" + str(i))
        request_movement("move_over_block", i) 
        rate.sleep()
        request_movement("open_gripper", -1)
        rate.sleep()
        print("move to block" + str(i+2))
        request_movement("move_to_block",i+2)
        rate.sleep()
    working = False
    rospy.set_param('/configuration',"stacked_ascending")
    print("Stack Ascending Completed")
        

def stack_descending():
    # Code for stack_descending blocks with stacked ascending orientation
    global working
    working = True
    rospy.set_param("/current_mode", "stack_descending")
    num_blocks = rospy.get_param("/num_blocks")
    rate = rospy.Rate(.5)
    for i in range(num_blocks-1,-1,-1):
        
        request_movement("close_gripper", -1)
        rate.sleep()

        if i == num_blocks-1:
            print("move over block" +str(0))
            request_movement("move_over_block", 0) # virtual block (table)
        else:
            print("move over block" + str(i+2))
            request_movement("move_over_block", i+2) 
        rate.sleep()
        request_movement("open_gripper", -1)
        rate.sleep()
        print("move to block" + str(i))
        request_movement("move_to_block", i)
        rate.sleep()


    

    working = False
    print("Stack Descending Completed")
    rospy.set_param('/configuration',"stacked_descending")
    return

def odd_even():
    return

def dual_scatter():
    # Code for moving blocks with scattered orientation
    return

def dual_stack_ascending():
   return
        

def dual_stack_descending():
    # Code for stack_descending blocks with stacked ascending orientation
    rate = rospy.Rate(.5)


    request_movement("close_gripper", -1)
    rate.sleep()
    request_movement("close_gripper", 1)
    rate.sleep()
    request_movement("move_over_block", 0)
    rate.sleep()
    request_movement("move_over_block", 4)
    rate.sleep()
    request_movement("open_gripper", -1)
    rate.sleep()
    request_movement("open_gripper", 1)
    rate.sleep()
    request_movement("move_to_block", 2)
    rate.sleep()
    request_movement("move_to_block", 1)
        

        



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
    


