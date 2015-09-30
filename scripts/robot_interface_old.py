#!/usr/bin/env python
import rospy
from njllrd_proj1.srv import *
from njllrd_proj1.msg import * 
import baxter_interface
from std_msgs.msg import String
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

gripper = None
left_gripper = None
first_run = None
initial_pose = None
left_initial_pose = None
pub = None
limb = None
left_limb = None
finger_length = None
block_height = None
msg = None
xpos = None
ypos = None
zpos = None
xcounter = 1
ycounter = 1
move_left_limb = -1 # -1 is for moving right arm, 1 is for moving left arm
stack_switch = 1
dual_arm_mode = None

def handle_move_robot(req):
    
    global dual_arm_mode

    if rospy.get_param('/dual_arm_mode') == True:
        dual_arm_mode = True
    else: 
        dual_arm_mode = False

    global first_run
    global initial_pose
    global left_initial_pose
    global xpos
    global ypos
    global zpos
    global xcounter
    global ycounter
    global stack_switch

    action = req.action
    target = req.target

    num_blocks = rospy.get_param('/num_blocks')

    if first_run == 1:
        setup_block_positions()
        first_run = 0

############################# OPEN GRIPPER #####################################
    if action == 'open_gripper':
        if target = 1:
            open_gripper('right')
        elif target = -1:   
            open_gripper('left')
        else:
            print "Invalid open gripper target"
            return False
        print "Opened Gripper"
        return True
########################### END OPEN GRIPPER ###################################


############################## CLOSE GRIPPER ###################################
    elif action == 'close_gripper':
        if target = 1:
            close_gripper('right')
        elif target = -1:
            close_gripper('left')
        else:
            print "Invalid close gripper target"
            return False
        print "Closed Gripper"
        return True
############################ END CLOSE GRIPPER #################################

    elif action == 'move_to_block'
        




############################### MOVE TO BLOCK ##################################
    elif action == 'move_to_block':
        if 1 == 0:  #msg.gripper_state == 1 or msg.stack == 0:
            return "False    positions"
        else:
            
            if (target>num_blocks or target<1):
                if dual_arm_mode:
                    msg.block_positions[0].x = msg.block_positions[0].x-stack_switch*.20
                      
                else:
                    msg.block_positions[0].y = msg.block_positions[0].y-stack_switch*.25
                msg.stack = 0
                stack_switch = -stack_switch
            else:
                msg.stack = 0 # We're now going to the initial pile
             
               
                

                # move up to "over end stack"
                value = initial_pose['position']
                current_pose = limb.endpoint_pose()
                current_pose = current_pose['position']
               
                if dual_arm_mode == True:

                    if target%2 == 0:
                        move_left_limb = -1
                    else:
                        move_left_limb = 1

                    if move_left_limb == 1:
                        new_pose = limb.Point(current_pose[0],current_pose[1]+move_left_limb*.20, value[2]+finger_length)
                        joints = request_kinematics(new_pose, initial_pose['orientation'],'left')
                        left_limb.move_to_joint_positions(joints)
                    else:
                        new_pose = limb.Point(current_pose[0],current_pose[1]+move_left_limb*.20, value[2]+finger_length)
                        joints = request_kinematics(new_pose, initial_pose['orientation'],'right')
                        limb.move_to_joint_positions(joints)
                else:
                    new_pose = limb.Point(current_pose[0],current_pose[1], value[2]+finger_length)
                    joints = request_kinematics(new_pose, initial_pose['orientation'],'right')
                    limb.move_to_joint_positions(joints,threshold = .004)
                
                # move up to "over initial stack"
                value = initial_pose['position']
                new_pose = limb.Point(msg.block_positions[target].x,msg.block_positions[target].y, value[2]+finger_length)
                joints = request_kinematics(new_pose, initial_pose['orientation'],')
                limb.move_to_joint_positions(joints,threshold = .004)
                
                # move down to next block

                targetblock = msg.block_positions[target]
                print("trying to get to")
                print(msg.block_positions[target])
                new_pose = limb.Point(targetblock.x, targetblock.y, targetblock.z)
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.move_to_joint_positions(joints,threshold = .004)
           
            return True
############################# END MOVE TO BLOCK ###############################










############################ MOVE OVER BLOCK ##################################
    # Drop block off at new location. Should change State, blockposition after completion
    elif action == 'move_over_block':
        if 1 == 0: #msg.gripper_state == 0 or msg.stack == 1:
            return False
        else:
            msg.stack = 1
            #print initial_pose
            
            value = initial_pose['position']
            current_pose = limb.endpoint_pose()
            current_pose = current_pose['position']
            
            
            if dual_arm_mode == True:
                even_odd_test = rospy.get_param("/num_blocks")%2
                if (target%2 == 0) and (target != 0 or even_odd_test):
                    move_left_limb = 1
                else:
                    move_left_limb = -1
            
            
            
           

                
                if move_left_limb == 1:
                    # move up to "over initial stack"
                    new_pose = limb.Point(current_pose[0],current_pose[1]+move_left_limb*.20, value[2]+finger_length)
                    print("moving left limb to: ")
                    print(new_pose)
                    joints = request_kinematics(new_pose, left_initial_pose['orientation'],'left')
                    print joints
                    left_limb.move_to_joint_positions(joints)
                    
                    
                    
                    
                else:
                    # move up to "over initial stack"
                    new_pose = limb.Point(current_pose[0],current_pose[1]+move_left_limb*.20, value[2]+finger_length)
                    print("moving right limb to: ")
                    print(new_pose)
                    joints = request_kinematics(new_pose, initial_pose['orientation'],'right')
                    limb.move_to_joint_positions(joints)
                    
            else:
                new_pose = limb.Point(current_pose[0],current_pose[1], value[2]+finger_length)
                joints = request_kinematics(new_pose, initial_pose['orientation'],'right')
                limb.move_to_joint_positions(joints,threshold = .004)
            #print initial_pose
            
            if target < 0:
                ################### SCATTER #####################
                # move over to "over desired location"
                if xcounter == 2:
                    xcounter = 0
                    ycounter = ycounter+1

                pose = limb.endpoint_pose()
                value = pose['position']
                new_pose = limb.Point(xpos+xcounter*.1,ypos+ycounter*.1, value[2])
                
                
                print("initial position:")
                print(pose['position'])
                print("position to go to")
                print(new_pose)
                print(xcounter)
                print(ycounter)
                
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.move_to_joint_positions(joints,threshold = .004)
                
                
                # Move down to desired spot
                new_pose = limb.Point(xpos+xcounter*.1,ypos+ycounter*.1, zpos)
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.move_to_joint_positions(joints,threshold = .004)
                


                print("initial position:")
                print(pose['position'])
                print("position to go to")
                print(new_pose)
                print(xcounter)
                print(ycounter)

                xcounter = xcounter+1
                
                if rospy.get_param('/configuration') == "stacked_descending":
                    print "change block %s x from %s to %s" %(-target,msg.block_positions[-target].x,new_pose[0])
                    msg.block_positions[-target].x = new_pose[0]
                    print "change block %s y from %s to %s" %(-target,msg.block_positions[-target].y,new_pose[1])
                    msg.block_positions[-target].y = new_pose[1]
                    print "change block %s z from %s to %s" %(-target,msg.block_positions[-target].z,new_pose[2])
                    msg.block_positions[-target].z = new_pose[2]
                elif rospy.get_param('/configuration') == "stacked_ascending":
                    num_blocks = rospy.get_param("/num_blocks")
                    print "change block %s x from %s to %s" %(num_blocks+1+target,msg.block_positions[num_blocks+1+target].x,new_pose[0])
                    msg.block_positions[num_blocks+1+target].x = new_pose[0]
                    print "change block %s y from %s to %s" %(num_blocks+1+target,msg.block_positions[num_blocks+1+target].y,new_pose[1])
                    msg.block_positions[num_blocks+1+target].y = new_pose[1]
                    print "change block %s z from %s to %s" %(num_blocks+1+target,msg.block_positions[num_blocks+1+target].z,new_pose[2])
                    msg.block_positions[num_blocks+1+target].z = new_pose[2]
               

            else:
                ############### NOT SCATTER ########################
                  
                targetblock = msg.block_positions[target]
	            # move over to "over end stack"
                pose = limb.endpoint_pose()
                value = pose['position']
                new_pose = limb.Point(targetblock.x,targetblock.y, value[2])
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.move_to_joint_positions(joints,threshold = .004)

                # lower block to be over "target" block
                num_blocks = rospy.get_param("/num_blocks")
                # num_blocks_moved = rospy.get_param("/num_blocks_moved")

                targetblock = msg.block_positions[target]
                newz = targetblock.z + block_height
                
                new_pose = limb.Point(targetblock.x,targetblock.y, newz)
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.move_to_joint_positions(joints,threshold = .004)
                # rospy.set_param("/num_blocks_moved",rospy.get_param("/num_blocks_moved")+1)

                current_pose = limb.endpoint_pose()
                print "Desired Pose"
                print(new_pose)
                print "Actual Pose"
                print(current_pose)
                
                if rospy.get_param("/current_mode") == "stack_ascending":
                    #change state of block we're moving, which in this case is the block we're putting a block on + 1
                    movedblock = msg.block_positions[target+1]
                    movedblock.x = targetblock.x
                    movedblock.y = targetblock.y
                    movedblock.z = targetblock.z + block_height
                    msg.block_positions[target+1] = movedblock
                elif rospy.get_param("/current_mode") == "stack_descending":
                    #change state of block we're moving, which in this case is the block we're putting a block on - 1
                    movedblock = msg.block_positions[target-1]
                    movedblock.x = targetblock.x
                    movedblock.y = targetblock.y
                    movedblock.z = targetblock.z + block_height
                    msg.block_positions[target-1] = movedblock

                print "moved over block %s" %(req.target)
                return True
########################### END MOVE OVER BLOCK ###############################




    else:
        return False
    # Returns false if action is invalid or action fails
    # else return False  print(initial_pose)
        


    print "gripper: %s, stack: %s" %(msg.gripper_state, msg.stack)
    # Returns true if action is valid and action is completed
    # Possible actions: open_gripper, close_gripper, move_to_block, move_over_block





## Move solely vertically
def move_up_vertical(limb):


## Move up and out
def move_out(limb, direction):


## Move solely sideways
def move_sideways(limb, direction):


############################ SETUP BLOCK POSITIONS ############################
def setup_block_positions():
     # Initialization of block positions, if this is the first request we're receiving
   
        
    # gets initial pose of arm
    initial_pose = limb.endpoint_pose()
    fpose = initial_pose
    
    # gets initial pose of left arm, if dual arm mode
    if dual_arm_mode == True:
        left_initial_pose = left_limb.endpoint_pose()
    

    # Sets position where scattering begins (nice if this can be adjusted)
    xpos = fpose['position'][0]-.2
    ypos = fpose['position'][1]+.1
    zpos = fpose['position'][2]-rospy.get_param("/num_blocks")*block_height+block_height

   
    # If we're in dual arm mode, new stack position should be offset in x
    # If we're in single arm mode, new stack position should be offset in y (more space)
    # New position is defined by location of block zero, which is set up first
    if dual_arm_mode:
        #set state of block 0 -- table underneath final stack
        block = blockposition()
        block.x = fpose['position'][0]+.20
        # moved over .25m
        block.y = fpose['position'][1]
        # moved down num_blocks*block_height
        block.z = fpose['position'][2]-rospy.get_param("/num_blocks")*block_height
        msg.block_positions.append(block)
        print(block)
    else:
        #set state of block 0 -- table underneath final stack
        block = blockposition()
        block.x = fpose['position'][0]
        # moved over .25m
        block.y = fpose['position'][1]+.25 
        # moved down num_blocks*block_height
        block.z = fpose['position'][2]-rospy.get_param("/num_blocks")*block_height
        msg.block_positions.append(block)
        print(block)
    

    # Set state of other blocks 
    # If our starting configuration is stacked descending, we can loop through and set the location of blocks using the index of our array
    # If stacked ascending, we need a different loop to make sure the blocks are in the right position in our block_positions array
    if rospy.get_param('/configuration') == "stacked_descending":
        #Set position of blocks if initial config = stacked descending
        for i in range(0,rospy.get_param("/num_blocks")):
            block = blockposition() # makes new instance of blockposition msg
            block.x = fpose['position'][0]  # sets x of block to fpose
            block.y = fpose['position'][1]  # sets y of block to fpose
            block.z = fpose['position'][2]-block_height*i  # sets z of block to fpose (varies with current block)
            msg.block_positions.append(block)   # adds block to block_positions array (in state msg)  
    elif rospy.get_param('/configuration') == "stacked_ascending":
        #Set position of blocks if initial config = stacked ascending
        # Take note of reverse loop to set blocks in correct index of block_positions array 
        for i in range(rospy.get_param("/num_blocks")-1,-1,-1):
            block = blockposition() # makes new instance of blockposition msg
            block.x = fpose['position'][0]  # sets x of block to fpose
            block.y = fpose['position'][1]  # sets y of block to fpose
            block.z = fpose['position'][2]-block_height*(i)  # sets z of block to fpose (varies with current block)
            msg.block_positions.append(block)   # adds block to block_positions array (in state msg)     
    
    print(msg.block_positions)
########################## END SETUP BLOCK POSITIONS ###########################






def move_over_block(limb, target):


def move_to_block(limb, target):





############################# OPEN GRIPPER #####################################
def open_gripper(limb):
    if limb == 'right':
        if msg.gripper_state == 0:
            print "Right gripper already open"
            return False
        else:
            gripper.open()
            msg.gripper_state = 0
            return True
    elif limb = 'left':
        if msg.left_gripper_state == 0:
            print "Left gripper already open"
            return False
        else:
            left_gripper.open()
            msg.left_gripper_state = 0
            return True
    else:
        print "Invalid limb type for open_gripper"
        return False
############################# END OPEN GRIPPER #####################################




############################## CLOSE GRIPPER ###################################
def close_gripper(limb):
    if limb == 'right':
        if msg.gripper_state == 1:
            print "Right gripper already closed"
            return False
        else:
            gripper.close()
            msg.gripper_state = 1
            return True
    elif limb = 'left':
        if msg.left_gripper_state == 1:
            print "Left gripper already closed"
            return False
        else:
            left_gripper.close()
            msg.left_gripper_state = 1
            return True
    else:
        print "Invalid limb type for close_gripper"
        return False
############################## END CLOSE GRIPPER ###############################




def request_kinematics(position, quaternion, limb):
    # rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK) 
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base') 
    poses = {
        'right': PoseStamped(
            header = hdr,
            pose = Pose(position,quaternion)
            ),
        'left': PoseStamped(
            header = hdr,
            pose = Pose(position,quaternion)
            ),
    }
    ikreq.pose_stamp.append(poses[limb])
    #print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if (resp.isValid[0]):
        #print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
 
    return False  



# Declares state of world at 1Hz using topic "state"
def broadcast_state():
    pub.publish(msg)
    return True
  



# Sets up all global variables, change parameters here  
def setup_variables():
    global gripper 
    gripper = baxter_interface.Gripper('right') #instantiate gripper
    global left_gripper 
    left_gripper = baxter_interface.Gripper('left') #instantiate left gripper
    global limb 
    limb = baxter_interface.Limb('right') #instantiate limb
    global left_limb 
    left_limb = baxter_interface.Limb('left') #instantiate left limb
    global initial_pose
    global first_run
    first_run = 1
    global finger_length
    finger_length = 0.05
    global block_height
    block_height = .045
    global msg
    msg = state()
    msg.gripper_state = 0 
    msg.stack = 0


# NODE START, communication start
def robot_interface():
    global pub
    pub = rospy.Publisher("state",state,queue_size=10)
    rospy.init_node('robot_interface')
    setup_variables()
    rate = rospy.Rate(1)
    s = rospy.Service('move_robot', move_robot, handle_move_robot)
    
    # gripper.calibrate() # calibrate gripper upon starting node
    while not rospy.is_shutdown():
        broadcast_state()
        rate.sleep()
    
    
    rospy.spin()


# MAIN
if __name__ == "__main__":
    robot_interface()
