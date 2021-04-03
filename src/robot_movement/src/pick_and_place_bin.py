#!/usr/bin/env python
import os
os.environ["ROS_NAMESPACE"] = "/robot1"
import sys
import rospy
import moveit_commander
from MoveGroup import MoveGroup

def mecademic_robot_basic_movement():
    # First initialize moveit_commander and rospy to interact with the MoveIt node and facilitate communication within the ros env
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)


    # Instantiate a MoveGroupCommander object.  This object is an interface
    # to one group of joints.  In this case the group refers to the joints of
    # the meca_arm.
    meca_arm_group = MoveGroup("meca_arm")

    # MoveGroup Commander Object for the mecademic hand.
    meca_fingers_group = MoveGroup("hand")

    # Ensure that the robot begins at its home position
    # Set a named joint configuration as the goal to plan for a move group.
    # Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
    meca_arm_group.move_to_home()

    # Ensure that the robot fingers are opened to pick up cube
    meca_fingers_group.move_via_joint_values([0.040, 0.040])


    # Ensure that the robot fingers are opened to pick up cube
    meca_arm_group.move_via_joint_values([-3.12414])


    #Cartesian path movement to pre grasp position
    meca_arm_group.relative_cartesian_movement([-.1, -999, -.4])

    # Close the mecademic robot fingers to pick cube up.
    meca_fingers_group.move_via_joint_values([0.00, 0.00])

    # Place the robot to its home position to begin place movement
    meca_arm_group.move_to_home()

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        print("running Program")
        mecademic_robot_basic_movement()
    except rospy.ROSInterruptException:
        print("Error occurred")
        pass
