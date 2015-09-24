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
first_run = None
initial_pose = None
pub = None
limb = None
finger_length = None
block_height = None
msg = None


def handle_move_robot(req):
    global first_run
    global initial_pose
    #print initial_pose
    if first_run == 1:
        initial_pose = limb.endpoint_pose()
        fpose = initial_pose
        
        #set state of block 0 -- table underneath final stack
        block = blockposition()
        block.x = fpose['position'][0]
        # moved over .25m
        block.y = fpose['position'][1]+.25 
        # moved down num_blocks*block_height
        block.z = fpose['position'][2]-rospy.get_param("/num_blocks")*block_height
        msg.block_positions.append(block)
        #set state of other blocks (THIS PART IS MESSING UP INITIAL_POSE SOMEHOW)
        """for i in range(0,rospy.get_param("/num_blocks")):
            print(i)
            block = blockposition()
            block.x = fpose['position'][0]
            block.y = fpose['position'][1]
            block.z = fpose['position'][2]
            msg.block_positions.append(block)
            value = fpose['position']
            fnew_position = limb.Point(value[0], value[1], value[2]-block_height)
            fpose['position'] = fnew_position
            print(fpose)
        """
        
        first_run = 0
    
        
    action = req.action
    target = req.target
    #print initial_pose
    print "gripper: %s, stack: %s" %(msg.gripper_state, msg.stack)
    # Returns true if action is valid and action is completed
    # Possible actions: open_gripper, close_gripper, move_to_block, move_over_block
   
    if action == 'open_gripper':
        if msg.stack == 0 or msg.gripper_state == 0: 
            return False
        else:
            msg.gripper_state = 0
            gripper.open()
            rate = rospy.Rate(.5)
            rate.sleep()
            print "opened gripper!"
            return True

    elif action == 'close_gripper':
        if msg.gripper_state == 1 or msg.stack == 1:
            return False
        else:
            msg.gripper_state = 1
            gripper.close()
            rate = rospy.Rate(.5)
            rate.sleep()
            print "CLOSEd gripper!"
            return True

    elif action == 'move_to_block':
        if msg.gripper_state == 1 or msg.stack == 0:
            return False
        else:
            msg.stack = 0
            # move up to "over end stack"
            value = initial_pose['position']
            new_pose = limb.Point(value[0],value[1]+.25, value[2]+finger_length)
            joints = request_kinematics(new_pose, initial_pose['orientation'])
            limb.move_to_joint_positions(joints)
            
            # move up to "over initial stack"
            value = initial_pose['position']
            new_pose = limb.Point(value[0],value[1], value[2]+finger_length)
            joints = request_kinematics(new_pose, initial_pose['orientation'])
            limb.move_to_joint_positions(joints)
            
            # move down to next block
            value = initial_pose['position']
            new_pose = limb.Point(value[0],value[1], value[2]-rospy.get_param("/num_blocks_moved")*block_height)
            joints = request_kinematics(new_pose, initial_pose['orientation'])
            limb.move_to_joint_positions(joints)
            
            return True

    elif action == 'move_over_block':
        if msg.gripper_state == 0 or msg.stack == 1:
            return False
        else:
            msg.stack = 1
            #print initial_pose
            # move up to "over initial stack"
            value = initial_pose['position']
            new_pose = limb.Point(value[0],value[1], value[2]+finger_length)
            joints = request_kinematics(new_pose, initial_pose['orientation'])
            limb.move_to_joint_positions(joints)
            #print initial_pose
            
	        # move over to "over end stack"
            pose = limb.endpoint_pose()
            value = pose['position']
            new_pose = limb.Point(value[0],value[1] + .25, value[2])
            joints = request_kinematics(new_pose, initial_pose['orientation'])
            limb.move_to_joint_positions(joints)

            # lower block to be over "target" block
            num_blocks = rospy.get_param("/num_blocks")
            num_blocks_moved = rospy.get_param("/num_blocks_moved")
            pose = limb.endpoint_pose()
            value = pose['position']
            new_pose = limb.Point(value[0],value[1], value[2]-block_height*(num_blocks-num_blocks_moved-1)-finger_length)
            joints = request_kinematics(new_pose, initial_pose['orientation'])
            limb.move_to_joint_positions(joints)
            rospy.set_param("/num_blocks_moved",rospy.get_param("/num_blocks_moved")+1)
           
            print "moved over block %s" %(req.target)
            return True
    else:
        return False


    # Returns false if action is invalid or action fails
    # else return False

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
    global gripper # declare global gripper
    gripper = baxter_interface.Gripper('right') #instantiate gripper
    global limb # declare global limb
    limb = baxter_interface.Limb('right') #instantiate limb
    global initial_pose
    global first_run
    first_run = 1
    global finger_length
    finger_length = 0.05
    global block_height
    block_height = .0445
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
