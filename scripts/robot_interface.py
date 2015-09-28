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

# Handler needs to be changed for two arm version
# I think we should have the same handler with a directional component based on the arm that's supposed to be moved

def handle_move_robot(req):
    
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

    # Initialization of block positions, if this is the first request we're receiving
    if first_run == 1:
        
        # gets initial pose of arm
        initial_pose = limb.endpoint_pose()
        fpose = initial_pose
        
        # gets initial pose of left arm, if dual arm mode
        if dual_arm_mode == True:
            left_initial_pose = left_limb.endpoint_pose()
        

        xpos = fpose['position'][0]-.2
        ypos = fpose['position'][1]+.1
        zpos = fpose['position'][2]-rospy.get_param("/num_blocks")*block_height+block_height

        #set state of block 0 -- table underneath final stack
        block = blockposition()
        block.x = fpose['position'][0]
        # moved over .25m
        block.y = fpose['position'][1]+.25 
        # moved down num_blocks*block_height
        block.z = fpose['position'][2]-rospy.get_param("/num_blocks")*block_height
        msg.block_positions.append(block)
        print(block)
        #set state of other blocks 


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
        first_run = 0 # makes sure we don't go into initialization step again
        
    action = req.action
    target = req.target
   
    print "gripper: %s, stack: %s" %(msg.gripper_state, msg.stack)
    # Returns true if action is valid and action is completed
    # Possible actions: open_gripper, close_gripper, move_to_block, move_over_block
   
    if action == 'open_gripper':
        # Shouldn't work if we're at the old stack, or the gripper is already open
        if msg.stack == 0 or msg.gripper_state == 0: 
            return False
        else:
            msg.gripper_state = 0 # Sets gripper state to open
            if move_left_limb == 1:
                left_gripper.open()
            else:
                gripper.open()

            move_left_limb = -move_left_limb
            # Sleeps for a bit, to allow the gripper to open. Make sure this matches sleep in controller
            rate = rospy.Rate(.5)
            rate.sleep()
            print "opened gripper!"
            return True

    elif action == 'close_gripper':
        # Shouldn't work if we're at the new stack, or the gripper is already closed
        if msg.gripper_state == 1 or msg.stack == 1:
            return False
        else:
            msg.gripper_state = 1 # Sets gripper state to closed
            if move_left_limb == 1
                left_gripper.close()
            else: 
                gripper.close()
             # Sleeps for a bit, to allow the gripper to close. Make sure this matches sleep in controller
            rate = rospy.Rate(.5)
            rate.sleep()
            print "CLOSEd gripper!"
            return True

    elif action == 'move_to_block':
        if msg.gripper_state == 1 or msg.stack == 0:
            return Falsepositions
        else:
            num_blocks = rospy.get_param('/num_blocks')
            if (target>num_blocks or target<1):
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
                    new_pose = limb.Point(current_pose[0],current_pose[1]+move_left_limb*.20, value[2]+finger_length)
                    joints = request_kinematics(new_pose, initial_pose['orientation'])
                    limb.set_joint_positions(joints)
                else:
                    new_pose = limb.Point(current_pose[0],current_pose[1], value[2]+finger_length)
                    joints = request_kinematics(new_pose, initial_pose['orientation'])
                    limb.move_to_joint_positions(joints,threshold = .004)
                
                # move up to "over initial stack"
                value = initial_pose['position']
                new_pose = limb.Point(msg.block_positions[target].x,msg.block_positions[target].y, value[2]+finger_length)
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.move_to_joint_positions(joints,threshold = .004)
                
                # move down to next block

                targetblock = msg.block_positions[target]
                print("trying to get to")
                print(msg.block_positions[target])
                new_pose = limb.Point(targetblock.x, targetblock.y, targetblock.z)
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.move_to_joint_positions(joints,threshold = .004)
            
            
            return True

    # Drop block off at new location. Should change State, blockposition after completion
    elif action == 'move_over_block':
        if msg.gripper_state == 0 or msg.stack == 1:
            return False
        else:
            msg.stack = 1
            #print initial_pose
            # move up to "over initial stack"
            value = initial_pose['position']
            current_pose = limb.endpoint_pose()
            current_pose = current_pose['position']
            if dual_arm_mode == True:
                new_pose = limb.Point(current_pose[0],current_pose[1]+move_left_limb*.20, value[2]+finger_length)
                joints = request_kinematics(new_pose, initial_pose['orientation'])
                limb.set_joint_positions(joints)
            else:
                new_pose = limb.Point(current_pose[0],current_pose[1], value[2]+finger_length)
                joints = request_kinematics(new_pose, initial_pose['orientation'])
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
                #####CHANGE BLOCK POSITIONS IN STATE!!!

            else:
                ############### NOT SCATTER ########################
	            # move over to "over end stack"
                pose = limb.endpoint_pose()
                value = pose['position']
                new_pose = limb.Point(msg.block_positions[0].x,msg.block_positions[0].y, value[2])
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
    else:
        return False
    # Returns false if action is invalid or action fails
    # else return False  print(initial_pose)
        

    print "gripper: %s, stack: %s" %(msg.gripper_state, msg.stack)
    # Returns true if action is valid and action is completed
    # Possible actions: open_gripper, close_gripper, move_to_block, move_over_block











def request_kinematics(position, quaternion):
    # rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK) 
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base') 
    poses = {
        'right': PoseStamped(
            header = hdr,
            pose = Pose(position,quaternion)
        )
    }
    ikreq.pose_stamp.append(poses['right'])
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
 
    return 0  


# Declares state of world at 1Hz using topic "state"
def broadcast_state():
    pub.publish(msg)
    return True
    
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

if __name__ == "__main__":
    robot_interface()
    




""" Move following two lines to whenever world state is initialized. 
    rospy.get_param('/global_num_blocks')
    rospy.get_param('/global_configuration') 
    Options: scattered, stacked_ascending, stacked_descending
"""
