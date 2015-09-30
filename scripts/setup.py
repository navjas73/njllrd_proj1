import rospy
import baxter_interface
import time

rospy.init_node('interface')

gripper = baxter_interface.Gripper('right')
limb = baxter_interface.Limb('right')
llimb = baxter_interface.Limb('left')
limb.endpoint_pose()
llimb.endpoint_pose()
gripper.calibrate()
limb.move_to_neutral()

# rosrun baxter_tools enable_robot.py -e

