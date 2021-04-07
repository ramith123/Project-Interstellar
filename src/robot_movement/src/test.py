#!/usr/bin/env python
import os
os.environ["ROS_NAMESPACE"] = "/robot1"
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



if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)
    pp = Pick_Place()
    # pp.back_to_home()
    pp.move_joint_hand(0.04)
    
    
    object_name = pp.get_object_list()[0]
    # get object pose
    # print(object_name)
    boxPose = pp.get_object_p(object_name)
    pp.scene.add_box(object_name, boxPose)
    # print(boxPose)
    # pose = pp.get_object_pose(object_name)
    
    
    # print(boxPose)
    # pp.scene.add_box(object_name,boxPose.pose)
    # print(pp.scene.get_objects())
    # print("done")
    # print(pp.scene.get_objects())
    # generate grasp message and pick it up
    # parameters WIDTH and LENGTH need to be tuned according to the object and grasping pose
    WIDTH = 0.04
    LENGTH = 0.2
    # print(boxPose.pose.position)
    grasp = pp.generate_grasp(object_name, "horizontal", boxPose.pose.position, WIDTH, length=LENGTH)

    # print(grasp)

    # pp.pickup(object_name, [grasp])
    pp.clean_scene(object_name)


    # choose target position and place the object
    # target_name = "storage"
    # place_position = pp.get_target_position(target_name)
    # pp.place("vertical", place_position)

    # pose = pp.arm.get_current_pose()
    # print(pose)

    #Forward K test
    # pp.move_joint_arm(radians(-55),radians(-69),radians(22),radians(-127),radians(-80),radians(145))
    # Inverse K Test
    # pose = pp.pose2msg(0,0,0,0.373586,0,0.009648)
    print(grasp.grasp_pose.pose)
    pp.move_pose_arm(grasp.grasp_pose.pose)