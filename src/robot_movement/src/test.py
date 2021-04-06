#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from pap_git import Pick_Place
from math import radians




if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)
    pp = Pick_Place()
    pp.back_to_home()
    pp.move_joint_arm(radians(-55),radians(-69),radians(22),radians(-127),radians(-80),radians(145))
