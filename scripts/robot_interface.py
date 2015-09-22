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




def handle_move_robot(req):
    if init == 0:
        initial_pose = limb.endpoint_pose()
        pose = initial_pose
        print(initial_pose)
        #set state of blocks
        #set state of block 0 # table underneath final stack
        initialization = 1
        
    action = req.action
    target = req.target
    
    print "gripper: %s, stack: %s" %(msg.gripper_state, msg.stack)
    # Returns true if action is valid and action is completed
    # Possible actions: open_gripper, close_gripper, move_to_block, move_over_block
    print "entered move_robot_callback"
    if action == 'open_gripper':
        if msg.stack == 0 or msg.gripper_state == 0: 
            return False
        else:
            msg.gripper_state = 0
            gripper.open()
            print "opened gripper!"
            return True
    elif action == 'close_gripper':
        if msg.gripper_state == 1 or msg.stack == 1:
            return False
        else:
            msg.gripper_state = 1
            gripper.set_moving_force(30)
            gripper.set_holding_force(30)
            gripper.command_position(0)
            print "CLOSEd gripper!"
            return True
    elif action == 'move_to_block':
        if msg.gripper_state == 1 or msg.stack == 0:
            return False
        else:
            msg.stack = 0
                # do something
            return True
    elif action == 'move_over_block':
        if msg.gripper_state == 0 or msg.stack == 1:
            return False
        else:
            msg.stack = 1
            new_pose = pose
            value = new_pose['position']
            new_pose = limb.Point(value[0],value[1], value[2]+finger_length)
            joints = request_kinematics(new_pose, initial_pose['orientation'])
            limb.move_to_joint_positions(joints)            
            #for key in new_pose:
            #    if key == "position":
            #        value = new_pose.get(key)   
            #        value['z'] = value['z'] + finger_length
            #        new_pose[key] = value
            
            # move up finger height
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
    print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
 
    return 0  


# Declares state of world at 1Hz using topic "state"
def broadcast_state():
    return True

def robot_interface():
    pub = rospy.Publisher("state",state,queue_size=10)
    rospy.init_node('robot_interface')
    rate = rospy.Rate(1)
    global msg
    msg = state()
    msg.gripper_state = 0 
    msg.stack = 0
    s = rospy.Service('move_robot', move_robot, handle_move_robot)
    global gripper # declare global gripper
    gripper = baxter_interface.Gripper('right') #instantiate gripper
    global limb # declare global limb
    limb = baxter_interface.Limb('right') #instantiate limb
    global initial_pose
    global init
    init = 0
    print init
    global finger_length
    finger_length = .1
    gripper.calibrate() # calibrate gripper upon starting node
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
