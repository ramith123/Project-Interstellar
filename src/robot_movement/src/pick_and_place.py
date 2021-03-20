#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from MoveGroup import MoveGroup


def mecademic_robot_basic_movement():
    # First initialize moveit_commander and rospy to interact with the MoveIt node and facilitate communication within the ros env
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)

    meca_arm_group = MoveGroup("meca_arm")
    meca_fingers_group = MoveGroup("hand")

    meca_arm_group.move_to_home()

    meca_fingers_group.move_via_joint_values([0.040, -1])

    meca_arm_group.cartesian_movement([0.05, -999, -0.4])

    meca_fingers_group.move_via_joint_values([0.00, -1])

    meca_arm_group.move_via_joint_values([0, 0, 0, 0, 0, 0])

    meca_arm_group.move_via_joint_values([1.5708])

    meca_arm_group.cartesian_movement([-999, 0.05, -0.4])

    meca_fingers_group.move_via_joint_values([0.040, -1])

    meca_arm_group.cartesian_movement([-999, 0, 0.4])

    meca_arm_group.move_via_joint_values([0, 0, 0, 0, 0, 0])

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        mecademic_robot_basic_movement()
    except rospy.ROSInterruptException:
        pass
