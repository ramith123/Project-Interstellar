#!/usr/bin/env python
import os
os.environ["ROS_NAMESPACE"] = "/robot2"
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
    roll = round(float(positions[4]), 5)
    pitch = round(float(positions[5]), 5)
    yaw = round(float(positions[6]), 5)
    w = round(float(positions[7]), 5)
    print("I heard %f %f %f %f %f %f", x, y, z, roll, pitch, yaw, w)
    return [[x, y, z], [roll, pitch, yaw, w]]


def add_offsets(cube_location):
    y_axis_position = cube_location[0][1]
    x_axis_position = cube_location[0][0]
    cube_location[0][2] -= 1

    if -0.10 < y_axis_position < 0.1:
        if x_axis_position <= 0.31:
            cube_location[0][0] -= 0.15
        elif x_axis_position > 0.35:
            cube_location[0][0] += 0.05
        return cube_location

    if y_axis_position > 0.1:
        cube_location[0][1] += (cube_location[0][1] * 4)

    elif y_axis_position < -0.10:
        cube_location[0][1] -= (cube_location[0][1] * -4)

    if x_axis_position > 0.3:
        cube_location[0][0] += (cube_location[0][0] * 2)

    elif x_axis_position <= 0.3:
        cube_location[0][0] -= (cube_location[0][0] * 2)

    return cube_location


def mecademic_robot_basic_movement():
    # First initialize moveit_commander and rospy to interact with the MoveIt node and facilitate communication within the ros env
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)

    msg = rospy.wait_for_message("/robot1/state_of_sequence1", String)
    print(msg.data)

    # rospy.Subscriber("locations", String, callback)
    msg = rospy.wait_for_message("/locations", String)


    orignal_cube_location = clean_message(msg)
    new_cube_location = add_offsets(orignal_cube_location)
    # orignal_cube_location[0][0] += -.8
    # orignal_cube_location[0][2] += .1
    

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

    
    meca_arm_group.move_via_joint_values([-3.10669])

    meca_arm_group.relative_cartesian_movement([-.12, -999, -.05])

    # # Cartesian path movement to pre grasp position
    # meca_arm_group.absolute_cartesian_movement(
    #     orignal_cube_location[0], orignal_cube_location[1])

    # Close the mecademic robot fingers to pick cube up.
    meca_fingers_group.move_via_joint_values([0.00, 0.00])


    meca_arm_group.move_to_home()

    #Cartesian path movement to place position
    meca_arm_group.relative_cartesian_movement([.1, -999, -.4])

    meca_fingers_group.move_via_joint_values([0.040, 0.040])

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
