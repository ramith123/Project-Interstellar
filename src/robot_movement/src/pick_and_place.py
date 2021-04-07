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
    roll = round(float(positions[4]), 5)
    pitch = round(float(positions[5]), 5)
    yaw = round(float(positions[6]), 5)
    w = round(float(positions[7]), 5)
    # print("I heard %s %f %f %f %f %f %f", positions[0], x, y, z, roll, pitch, yaw, w)
    return [positions[0], [x, y, z], [roll, pitch, yaw, w]] 


def gather_cube_positions():

    orignal_cube_locations = {}
    boxes = ['box1', 'box2']

    for box in boxes: 
        msg = rospy.wait_for_message("/locations", String)
        cube_location = clean_message(msg)
        while cube_location[0] != box: 
            msg = rospy.wait_for_message("/locations", String)
            cube_location = clean_message(msg)
        orignal_cube_locations[box] = {'position1':[cube_location[1], cube_location[2]]} 

    return orignal_cube_locations



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

    # Instantiate a MoveGroupCommander object.  This object is an interface
    # to one group of joints.  In this case the group refers to the joints of
    # the meca_arm.
    meca_arm_group = MoveGroup("meca_arm")

    # MoveGroup Commander Object for the mecademic hand.
    meca_fingers_group = MoveGroup("hand")

    cube_locations = gather_cube_positions()
    cube_locations['box1']['position2'] = [-999, 0.05, -0.4]
    cube_locations['box2']['position2'] = [-0.3, 0.05, -0.6]
    cube_locations['box1']['position3'] = [-999, 0, 0.25]
    cube_locations['box2']['position3'] = [0, 0, 0.5]
    boxes = ['box2']



    # Movement to storage area 
    for box in boxes: 

        cube_locations[box]['position1'] = add_offsets(cube_locations[box]['position1'])

        # Ensure that the robot begins at its home position
        # Set a named joint configuration as the goal to plan for a move group.
        # Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
        meca_arm_group.move_to_home()

        # Ensure that the robot fingers are opened to pick up cube
        meca_fingers_group.move_via_joint_values([0.040, -1])

        # Cartesian path movement to pre grasp position
        meca_arm_group.absolute_cartesian_movement(cube_locations[box]['position1'][0], cube_locations[box]['position1'][1])

        # Close the mecademic robot fingers to pick cube up.
        meca_fingers_group.move_via_joint_values([0.00, -1])

        # Place the robot to its home position to begin place movement
        meca_arm_group.move_to_home()

        # Rotate the robot 90%
        meca_arm_group.move_via_joint_values([1.5708])

        # Perform the pre-place movement
        meca_arm_group.relative_cartesian_movement(cube_locations[box]['position2'])

        # Open the mecademic robot fingers to place cube.
        meca_fingers_group.move_via_joint_values([0.040, -1])

        # move robot up a bit to clear the cube
        meca_arm_group.relative_cartesian_movement([0, 0 , 0.2])

        meca_arm_group.move_to_home()

    #Movement to bin area 

    for box in boxes: 

        # Rotate the robot 90%
        meca_arm_group.move_via_joint_values([1.5708])

        # Ensure that the robot fingers are opened to pick up cube
        meca_fingers_group.move_via_joint_values([0.040, -1])

        # Cartesian path movement to pre grasp position
        meca_arm_group.relative_cartesian_movement(cube_locations[box]['position2'])

        # Close the mecademic robot fingers to pick cube up.
        meca_fingers_group.move_via_joint_values([0.00, -1])

        # move robot upwards to avoid hitting cubes below
        meca_arm_group.relative_cartesian_movement(cube_locations[box]['position3'])

        # Place the robot to its home position to begin place movement
        meca_arm_group.move_to_home()

        # Perform the pre-place movement
        meca_arm_group.absolute_cartesian_movement(cube_locations[box]['position1'][0], cube_locations[box]['position1'][1])

        # Open the mecademic robot fingers to place cube.
        meca_fingers_group.move_via_joint_values([0.040, -1])

        # move robot up a bit to clear the cube
        meca_arm_group.relative_cartesian_movement([0, 0 , 0.2])


        meca_arm_group.move_to_home()
    
    

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        mecademic_robot_basic_movement()
    except rospy.ROSInterruptException:
        pass
