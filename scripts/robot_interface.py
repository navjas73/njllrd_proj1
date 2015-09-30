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

raised_height = None
out_distance = None
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
dualcounter = 1

def handle_move_robot(req):
    
    global dual_arm_mode

    if rospy.get_param('/dual_arm_mode') == True:
        dual_arm_mode = True
    else: 
        dual_arm_mode = False

    global first_run

    
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
        if target == -1:
            success = open_gripper('right')
        elif target == 1:   
            success = open_gripper('left')
        else:
            print "Invalid open gripper target"
            return False
        if success:
            print "Opened Gripper"
        return success
########################### END OPEN GRIPPER ###################################


############################## CLOSE GRIPPER ###################################
    elif action == 'close_gripper':
        if target == -1:
            success = close_gripper('right')
        elif target == 1:
            success = close_gripper('left')
        else:
            print "Invalid close gripper target"
            return False
        if success:    
            print "Closed Gripper"
        return success
############################ END CLOSE GRIPPER #################################


############################### MOVE TO BLOCK ##################################
    elif action == 'move_to_block':
        if (target>num_blocks or target<1):
            if dual_arm_mode:
                msg.block_positions[0].x = msg.block_positions[0].x-stack_switch*.20
                  
            else:
                msg.block_positions[0].y = msg.block_positions[0].y-stack_switch*.25
            msg.stack = 0
            stack_switch = -stack_switch
            success = True
        else:
            if dual_arm_mode:
                if target%2 == 0:
                    targetlimb = 'right'
                else:
                    targetlimb = 'left'
            else:
                targetlimb = 'right'
            success = move_to_block(targetlimb, target)
            if success:
                print "Moved to block: " + str(target) + " with limb" + targetlimb
            else:
                print "Failed to move to block: " + str(target) + " with limb" + targetlimb
        return success
############################# END MOVE TO BLOCK ###############################


############################ MOVE OVER BLOCK ##################################
    # Drop block off at new location. Should change State, blockposition after completion
    elif action == 'move_over_block':
        if dual_arm_mode == True:
            even_odd_test = rospy.get_param("/num_blocks")%2
            configuration = rospy.get_param('/configuration')
            if (target%2 == 0) and (target != 0):
                targetlimb = 'left'
            elif (target == 0) and not((configuration == "stacked_ascending") and not(even_odd_test)):
                targetlimb = 'left'
            else:
                targetlimb = 'right'
        else:
            targetlimb = 'right'
        print targetlimb
        success = move_over_block(targetlimb, target)
        if success:
            print "Moved over block: " + str(target) + " with limb" + targetlimb
        else:
            print "Failed to move over block: " + str(target) + " with limb" + targetlimb
        
        return success
########################### END MOVE OVER BLOCK ###############################
    
    else:
        print "Invalid action request"
        return False
    # Returns false if action is invalid or action fails
        








################################## MOVEMENT FUNCTIONS ##########################

## Move up vertically only
def move_up(side):
    success = True

    if side == 'right':
        current_pose = limb.endpoint_pose()
        current_pose = current_pose['position']
        new_pose = limb.Point(current_pose[0],current_pose[1], raised_height)
        joints = request_kinematics(new_pose, initial_pose['orientation'],'right')
        try:
            limb.move_to_joint_positions(joints)
        except:
            success = False

    elif side == 'left':
        current_pose = left_limb.endpoint_pose()
        current_pose = current_pose['position']
        new_pose = limb.Point(current_pose[0],current_pose[1], raised_height)
        joints = request_kinematics(new_pose, left_initial_pose['orientation'],'left')
        
        try:
            left_limb.move_to_joint_positions(joints)
        except:
            success = False

    return success

## Move up and out
def move_up_and_out(side):
    if side == 'right':
        current_pose = limb.endpoint_pose()
        current_pose = current_pose['position']
        new_pose = limb.Point(current_pose[0],current_pose[1]-out_distance, raised_height)
        joints = request_kinematics(new_pose, initial_pose['orientation'], 'right')
        try:
            limb.move_to_joint_positions(joints)
        except:
            success = False
    elif side == 'left':
        current_pose = left_limb.endpoint_pose()
        current_pose = current_pose['position']
        new_pose = limb.Point(current_pose[0],current_pose[1]+out_distance, raised_height)
        joints = request_kinematics(new_pose, left_initial_pose['orientation'],'left')
        try:
            left_limb.move_to_joint_positions(joints)
        except:
            success = False
    return True


## Move over your target block
def move_over_target(side, target):
    if side == 'right':
        
        targetblock = msg.block_positions[target]
        pose = limb.endpoint_pose()
        value = pose['position']
        new_pose = limb.Point(targetblock.x,targetblock.y, value[2])
        joints = request_kinematics(new_pose, initial_pose['orientation'], 'right')
        limb.move_to_joint_positions(joints)

    
    elif side == 'left':
        
        targetblock = msg.block_positions[target]
        pose = left_limb.endpoint_pose()
        value = pose['position']
        new_pose = limb.Point(targetblock.x,targetblock.y, value[2])
        joints = request_kinematics(new_pose, left_initial_pose['orientation'], 'left')
        left_limb.move_to_joint_positions(joints)

    return True

## Lower over your target block
def lower_over_target(side, target):
    
    if side == 'right':
        
        targetblock = msg.block_positions[target]
        new_pose = limb.Point(targetblock.x,targetblock.y, targetblock.z + block_height)
        joints = request_kinematics(new_pose, initial_pose['orientation'], 'right')
        limb.move_to_joint_positions(joints)

    
    elif side == 'left':
        
        targetblock = msg.block_positions[target]
        new_pose = limb.Point(targetblock.x,targetblock.y, targetblock.z + block_height)
        joints = request_kinematics(new_pose, left_initial_pose['orientation'], 'left')
        left_limb.move_to_joint_positions(joints)

    return True

## Lower to your target block
def lower_to_target(side, target):

    if side == 'right':
        
        targetblock = msg.block_positions[target]
        new_pose = limb.Point(targetblock.x,targetblock.y, targetblock.z)
        joints = request_kinematics(new_pose, initial_pose['orientation'], 'right')
        limb.move_to_joint_positions(joints)

    
    elif side == 'left':
        
        targetblock = msg.block_positions[target]
        new_pose = limb.Point(targetblock.x,targetblock.y, targetblock.z)
        joints = request_kinematics(new_pose, left_initial_pose['orientation'], 'left')
        left_limb.move_to_joint_positions(joints)
    return True

def scatter_block(side, target):
    global xcounter
    global ycounter
    global dualcounter
    
    if xcounter == 2:
        xcounter = 0
        ycounter = ycounter+1

              
    move_up(side) 
    
    if side == 'right':
        current_pose = limb.endpoint_pose()
        current_pose = current_pose['position']
        # Move over to desired spot
        if dual_arm_mode == True:
            new_pose = limb.Point(current_pose[0],current_pose[1]-dualcounter*0.1,current_pose[2])
        else:
            new_pose = limb.Point(xpos+xcounter*.1,ypos-ycounter*.1, current_pose[2])
        joints = request_kinematics(new_pose, initial_pose['orientation'], 'right')
        limb.move_to_joint_positions(joints)

        
        # Move down to desired spot
        if dual_arm_mode == True:
            new_pose = limb.Point(current_pose[0],current_pose[1]-dualcounter*0.1,zpos)
        else:
            new_pose = limb.Point(xpos+xcounter*.1,ypos+ycounter*.1, zpos)
        joints = request_kinematics(new_pose, initial_pose['orientation'], 'right')
        limb.move_to_joint_positions(joints)

    elif side == 'left':
        
        current_pose = left_limb.endpoint_pose()
        current_pose = current_pose['position']
        # Move over to desired spot
        new_pose = limb.Point(current_pose[0],current_pose[1]+dualcounter*0.1,current_pose[2])
        joints = request_kinematics(new_pose, left_initial_pose['orientation'], 'left')
        left_limb.move_to_joint_positions(joints)

        # Move down to desired spot
        new_pose = limb.Point(current_pose[0],current_pose[1]+dualcounter*0.1,zpos)
        joints = request_kinematics(new_pose, left_initial_pose['orientation'], 'left')
        left_limb.move_to_joint_positions(joints)
                
               
    xcounter = xcounter+1
    dualcounter = dualcounter + 1
                
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

    return True
########################### END MOVEMENT FUNCTIONS #############################




############################ SETUP BLOCK POSITIONS ############################
def setup_block_positions():
     # Initialization of block positions, if this is the first request we're receiving
   
    global initial_pose
    global left_initial_pose
    global xpos
    global ypos
    global zpos
    global raised_height
    # gets initial pose of arm
    initial_pose = limb.endpoint_pose()
    fpose = initial_pose
    
    raised_height = fpose['position'][2]+finger_length

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

            # Set state of other blocks 
        # If our starting configuration is stacked descending, we can loop through and set the location of blocks using the index of our array
        # If stacked ascending, we need a different loop to make sure the blocks are in the right position in our block_positions array
        test = rospy.get_param('/num_blocks')%2

       

        if rospy.get_param('/configuration') == "stacked_descending":
            fpose = left_initial_pose
            #Set position of blocks if initial config = stacked descending
            for i in range(0,rospy.get_param("/num_blocks")):
                block = blockposition() # makes new instance of blockposition msg
                block.x = fpose['position'][0]  # sets x of block to fpose
                block.y = fpose['position'][1]  # sets y of block to fpose
                block.z = fpose['position'][2]-block_height*i  # sets z of block to fpose (varies with current block)
                msg.block_positions.append(block)   # adds block to block_positions array (in state msg)  
        elif rospy.get_param('/configuration') == "stacked_ascending":
            if test:
                fpose = left_initial_pose
            else:
                fpose = initial_pose

            #Set position of blocks if initial config = stacked ascending
            # Take note of reverse loop to set blocks in correct index of block_positions array 
            for i in range(rospy.get_param("/num_blocks")-1,-1,-1):
                block = blockposition() # makes new instance of blockposition msg
                block.x = fpose['position'][0]  # sets x of block to fpose
                block.y = fpose['position'][1]  # sets y of block to fpose
                block.z = fpose['position'][2]-block_height*(i)  # sets z of block to fpose (varies with current block)
                msg.block_positions.append(block)   # adds block to block_positions array (in state msg)     




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





########################### MOVE OVER BLOCK ####################################
def move_over_block(side, target):

    if target < 0:
        success = scatter_block(side, target)
        return success

    if dual_arm_mode:
        success1 = move_up_and_out(side)
    else:
        success1 = move_up(side)


    success2 = move_over_target(side, target)

    success3 = lower_over_target(side, target)
    
    print success1
    print success2
    print success3

    if success1 and success2 and success3:
        targetblock = msg.block_positions[target]
        ## SET NEW BLOCK POSITION
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

        success = True

    else:
        success = False

    return success
########################### END MOVE OVER BLOCK ####################################

########################### MOVE TO BLOCK ######################################
def move_to_block(side, target):

    if dual_arm_mode:
        success1 = move_up_and_out(side)
    else:
        success1 = move_up(side)

    success2 = move_over_target(side, target)

    success3 = lower_to_target(side, target)

    if success1 and success2 and success3:
        success = True
    else:
        success = False

    return success
############################# END MOVE TO BLOCK ################################




############################# OPEN GRIPPER #####################################
def open_gripper(side):
    if side == 'right':
        if msg.gripper_state == 0:
            print "Right gripper already open"
            return False
        else:
            gripper.open()
            msg.gripper_state = 0
            return True
    elif side == 'left':
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
############################# END OPEN GRIPPER #################################




############################## CLOSE GRIPPER ###################################
def close_gripper(side):
    if side == 'right':
        if msg.gripper_state == 1:
            print "Right gripper already closed"
            return False
        else:
            gripper.close()
            msg.gripper_state = 1
            return True
    elif side == 'left':
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




def request_kinematics(position, quaternion, side):
    # rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
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
    ikreq.pose_stamp.append(poses[side])
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
    global out_distance
    out_distance = .2

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
