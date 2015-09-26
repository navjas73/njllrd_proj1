import rospy
import baxter_interface
import time

rospy.init_node('interface')
time.sleep(3)
gripper = baxter_interface.Gripper('right')
limb = baxter_interface.Limb('right')
gripper.calibrate()
limb.move_to_neutral()

