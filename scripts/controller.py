#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
from njllrd_proj1.msg import *
import time
from std_msgs.msg import String

working = False
#fromPole = None
#withPole = None
#toPole = None

# When topic "command" is published to, reads block orientation mode and
#   starts relevant process
def mode_selection(data):
    dual_arm = rospy.get_param("/dual_arm_mode")
    configuration = rospy.get_param("/configuration")
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
            elif data.data == "tower_of_hanoi":
                #global fromPole
                #global withPole
                #global toPole
                # initialize towers with blocks that represent the table
                n = rospy.get_param("/num_blocks")
                fromPole = range(n,0,-1)
                print fromPole
                fromPole.insert(0,n+2)
                withPole = [0]
                toPole = [n+1]
                moveTower(n,fromPole,toPole,withPole)
                
            else: 
                print("Invalid mode")
        else:
            if data.data == "scatter":
                if configuration != "scattered":
                    dual_scatter()
                else:
                    print "Already scattered"
            elif data.data == "stack_ascending":
                if configuration != "stacked_ascending":
                    dual_stack_ascending()
                else:
                    print "Already stacked ascending"
            elif data.data == "stack_descending":
                if configuration != "stacked_descending":
                    dual_stack_descending()
                else:
                    print "Already stacked descending"
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
    request_movement("move_to_block", 1)
    rate.sleep()
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
    request_movement("move_to_block", num_blocks)
    rate.sleep()
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
    global working
    working = True
    rospy.set_param("/current_mode", "scatter")
    # Code for moving blocks with scattered orientation
    num_blocks = rospy.get_param("/num_blocks")
    rate = rospy.Rate(.5)
    #newRange = range(1,num_blocks+1,2)
    #newRange.insert(0,0)
    
    for i in range(1,num_blocks+1,2):
        request_movement("close_gripper", -1)
        rate.sleep()
        request_movement("close_gripper", 1)
        rate.sleep()
        if i < num_blocks + 1:
            request_movement("move_over_block", -i) # virtual block (table)
            rate.sleep()
        if i+1 < num_blocks + 1:
            request_movement("move_over_block", -i-1) # virtual block (table)
            rate.sleep()
        request_movement("open_gripper", -1)
        rate.sleep()
        request_movement("open_gripper", 1)
        rate.sleep()
        if rospy.get_param("/configuration") == "stacked_descending":
            if i+3 < num_blocks+1:
                request_movement("move_to_block",(i+3))
                rate.sleep()
            if i+2 < num_blocks+1:
                request_movement("move_to_block",(i+2))
                rate.sleep()
        elif rospy.get_param("/configuration") == "stacked_ascending":
            if num_blocks - i -2 < num_blocks+1:
                request_movement("move_to_block",(num_blocks - i -2))
                rate.sleep()
            if num_blocks - i -1 < num_blocks+1:
                request_movement("move_to_block",(num_blocks - i -1))
                rate.sleep()

    working = False
    print("Scatter Completed")
    rospy.set_param('/configuration',"scattered")

def dual_stack_ascending():
    global working
    rospy.set_param("/current_mode", "stack_ascending")
    working = True
    rate = rospy.Rate(0.5)
    num_blocks = rospy.get_param("/num_blocks")
    for i in range(1,num_blocks+1,2):
        request_movement("close_gripper",-1) 
        rate.sleep()
        request_movement("close_gripper",1)
        rate.sleep()
        if i-1 < num_blocks:
            request_movement("move_over_block",i-1)
            rate.sleep()
            request_movement("open_gripper", 1)
            rate.sleep()
        if i < num_blocks:
            request_movement("move_over_block",i)
            rate.sleep()        
            request_movement("open_gripper", -1)
            rate.sleep()

        if i+3 < num_blocks + 1:
            request_movement("move_to_block",i+3)
            rate.sleep()
        if i+2 < num_blocks + 1:
            request_movement("move_to_block",i+2)
            rate.sleep()
    print("Dual Stack Ascending Completed")  
    working = False
    rospy.set_param('/configuration',"stacked_ascending")      

def dual_stack_descending():
    global working
    rospy.set_param("/current_mode", "stack_descending")
    working = True
    # Code for stack_descending blocks with stacked ascending orientation
    rate = rospy.Rate(.5)
    num_blocks = rospy.get_param("/num_blocks")
    for i in range(num_blocks,-1,-2):
        request_movement("close_gripper",-1) 
        rate.sleep()
        request_movement("close_gripper",1)
        rate.sleep()
        if i == num_blocks:
            request_movement("move_over_block",0)
        elif i+1 > 1:
            request_movement("move_over_block",i+1)
        rate.sleep()
        if i > 1:
            request_movement("move_over_block",i)
            rate.sleep()        
        request_movement("open_gripper", -1)
        rate.sleep()
        request_movement("open_gripper", 1)
        rate.sleep()
        if i-3 > 0:
            request_movement("move_to_block",i-3)
            rate.sleep()
        if i-2 > 0:
            request_movement("move_to_block",i-2)
            rate.sleep()
    print("Dual Stack Descending Completed")
    working = False
    rospy.set_param('/configuration',"stacked_descending")

    return

def dual_odd_even():
    num_blocks = rospy.get_param("/num_blocks")
    configuration = rospy.get_param("/configuration")

    if configuration == "stacked_ascending":
        for i in range(num_blocks, -1,-1):
            request_movement("close_gripper",-1) 
            rate.sleep()
            request_movement("close_gripper",1)
            rate.sleep()
            if i == num_blocks and num_blocks%2 == 0:
                request_movement("move_over_block", 0)
            elif i == num_blocks:  
                request_movement("move_over_block", num_blocks+1)
            elif i == num_blocks-1 and num_blocks%2 == 0: 
                request_movement("move_over_block", num_blocks+1)
            elif i == num_blcoks-1:
                request_movement("move_over_block", 0)
            else:
                request_movement("move_over_block", i+2)

    elif configuration == "stacked_descending":
        for i in range(1,num_blocks):
            request_movement("close_gripper",-1) 
            rate.sleep()
            request_movement("close_gripper",1)
            rate.sleep()
            if i == 1:
                request_movement("move_over_block", num_blocks+1)
            elif i == 2:  
                request_movement("move_over_block", 0)
            else:
                request_movement("move_over_block", i-2)
        

    return True

def moveTower(height,fromPole,toPole,withPole):
    if height >= 1:
        moveTower(height-1,fromPole,withPole,toPole)
        [fromPole,toPole]=moveDisk(fromPole, toPole)
        moveTower(height-1,withPole,toPole,fromPole)

def moveDisk(fromPole, toPole):
    rate = rospy.Rate(.5)
    request_movement("move_to_block",fromPole[-1])
    rate.sleep()
    request_movement("close_gripper",-1)
    rate.sleep()
    request_movement("move_over_block",toPole[-1])
    rate.sleep()
    request_movement("open_gripper",-1)
    rate.sleep()
    toPole.append(fromPole.pop(-1))
    print fromPole
    print toPole
    return [fromPole,toPole]

def controller():
    rospy.init_node('controller')
    rospy.wait_for_service('move_robot')
    rospy.Subscriber('command', String, mode_selection)
    # rospy.Subscriber('state', state, read_state)
    
    # time.sleep(10)
    rospy.spin()

if __name__ == "__main__":
    controller()
    


