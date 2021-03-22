#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from MoveGroup import MoveGroup
from std_msgs.msg import String


def clean_message(message):
    unformated_positions = message.data.strip("\n")
    positions = unformated_positions.split(",")
    x = round(float(positions[1]), 2)
    y = round(float(positions[2]), 2)
    z = round(float(positions[3]), 2)
    print("I heard %f %f %f", x, y, z)
    return [x, y, z]


def mecademic_robot_basic_movement():
    # First initialize moveit_commander and rospy to interact with the MoveIt node and facilitate communication within the ros env
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)

    # rospy.Subscriber("locations", String, callback)

    msg = rospy.wait_for_message("locations", String)
    cube_position = clean_message(msg)

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
    meca_fingers_group.move_via_joint_values([0.040, -1])

    # Cartesian path movement to pre grasp position
    meca_arm_group.go_to_pose_goal(cube_position)

    # Close the mecademic robot fingers to pick cube up.
    meca_fingers_group.move_via_joint_values([0.00, -1])

    # Place the robot to its home position to begin place movement
    meca_arm_group.move_to_home()

    # Rotate the robot 90%
    meca_arm_group.move_via_joint_values([1.5708])

    # Perform the pre-place movement
    meca_arm_group.cartesian_movement([-999, 0.05, -0.4])

    # Open the mecademic robot fingers to place cube.
    meca_fingers_group.move_via_joint_values([0.040, -1])

    # move robot up a bit to clear the cube
    meca_arm_group.cartesian_movement([-999, 0, 0.3])

    meca_arm_group.move_to_home()

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        mecademic_robot_basic_movement()
    except rospy.ROSInterruptException:
        pass
