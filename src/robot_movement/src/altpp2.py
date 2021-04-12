#!/usr/bin/env python
import os
os.environ["ROS_NAMESPACE"] = "/robot2"
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from pap_git import Pick_Place
from math import radians
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool
VLENGTH = 0.112
LENGTH = 0.104

def publishSync(pubisher,value):
    msg = Bool()
    msg.data = value
    pubisher.publish(msg)

def cartesianMove(pp,pose):
    waypoints = []
    waypoints.append(pose.pose)

    (pose, fraction) = pp.arm.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.005,        # eef_step
                                    0)         # jump_threshold
    
    pp.arm.execute(pose, wait=True)
    

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place2', anonymous=True)
    syncPub = rospy.Publisher("/robot1SyncBool", Bool, queue_size=0)
    pp = Pick_Place()# print(pp.get_object_list())
    pp.back_to_home()
    
    pp.move_joint_hand(0.03)
    currentPose = pp.arm.get_current_pose()
    currentPose.pose.position.z -=0.25
    currentPose.pose.position.x +=0.025
    cartesianMove(pp,currentPose)
    
    pp.move_joint_hand(0.001)
    rospy.sleep(4)
    currentPose = pp.arm.get_current_pose()
    currentPose.pose.position.z +=0.1
    cartesianMove(pp,currentPose)
    publishSync(syncPub,True)
    rospy.wait_for_message("/robot2SyncBool",Bool)
    currentPose = pp.arm.get_current_pose()
    currentPose.pose.position.z -=0.1
    cartesianMove(pp,currentPose)
    pp.move_joint_hand(0.03)
    rospy.sleep(3)
    

    pp.move_joint_hand(0.001)
    rospy.sleep(4)
    currentPose = pp.arm.get_current_pose()
    currentPose.pose.position.z +=0.1
    cartesianMove(pp,currentPose)
    publishSync(syncPub,True)
    rospy.wait_for_message("/robot2SyncBool",Bool)
    rospy.sleep(2)
    currentPose = pp.arm.get_current_pose()
    currentPose.pose.position.z -=0.1
    cartesianMove(pp,currentPose)
    pp.move_joint_hand(0.03)
    rospy.sleep(3)

    currentPose = pp.arm.get_current_pose()
    currentPose.pose.position.z +=0.1
    cartesianMove(pp,currentPose)
    pp.back_to_home()
    
    syncPub.unregister()

