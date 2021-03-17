#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs


def mecademic_robot_basic_movement():

    # First initialize moveit_commander and rospy to interact with the MoveIt node and facilitate communication within the ros env
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place',
                    anonymous=True)

    # Instantiate a MoveGroupCommander object.  This object is an interface
    # to one group of joints.  In this case the group refers to the joints of
    # the meca_arm.
    meca_arm_group = moveit_commander.MoveGroupCommander("meca_arm")
    # MoveGroup Commander Object for the mecademic hand.
    meca_fingers_group = moveit_commander.MoveGroupCommander("hand")

    # Action clients to the ExecuteTrajectory action server
    meca_arm_client = actionlib.SimpleActionClient('execute_trajectory',
                                                   moveit_msgs.msg.ExecuteTrajectoryAction)

    # Ensure that the server is ready to receive request
    meca_arm_client.wait_for_server()

    meca_fingers_client = actionlib.SimpleActionClient('execute_trajectory',
                                                       moveit_msgs.msg.ExecuteTrajectoryAction)
    meca_fingers_client.wait_for_server()

    # Ensure that the robot begins at its home position
    # Set a named joint configuration as the goal to plan for a move group.
    # Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
    meca_arm_group.set_named_target("home pose")

    # Plan to the desired joint-space goal using the default planner (RRTConnect).
    meca_arm_home = meca_arm_group.plan()
    # Create a goal message object for the action server.
    meca_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    # Update the trajectory in the goal message.
    meca_arm_goal.trajectory = meca_arm_home

    # Send the goal to the action server.
    meca_arm_client.send_goal(meca_arm_goal)
    meca_arm_client.wait_for_result()

    # Ensure that the robot fingers are opened to pick up cube
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = meca_fingers_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = 0.040

    # Move to the specified joint goal
    meca_fingers_group.go(joint_goal, wait=True)

    # ensures that there is no residual movement
    meca_fingers_group.stop()

    # Cartesian path movement to pre grasp position
    waypoints = []
    # start with the current pose
    current_pose = meca_arm_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = meca_arm_group.get_current_pose()

    # create linear offsets to the current pose
    new_eef_pose = geometry_msgs.msg.Pose()

    # Manual offsets because we don't have the senser coordinated from the gazebo plugin to detect cubes yet.
    new_eef_pose.position.x = current_pose.pose.position.x + 0.05
    new_eef_pose.position.z = current_pose.pose.position.z - 0.4

    # Retain orientation of the current pose.
    new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

    waypoints.append(new_eef_pose)

    # We want the cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in cartesian
    # translation.  We will specify the jump threshold as 0.0, effectively
    # disabling it.
    fraction = 0.0
    for count_cartesian_path in range(0, 3):
        if fraction < 1.0:
            (plan_cartesian, fraction) = meca_arm_group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
        else:
            print('error')
            break

    meca_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    meca_arm_goal.trajectory = plan_cartesian
    meca_arm_client.send_goal(meca_arm_goal)
    meca_arm_client.wait_for_result()

    # Close the mecademic robot fingers to pick cube up.
    joint_goal = meca_fingers_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = 0.00
    meca_fingers_group.go(joint_goal, wait=True)
    meca_fingers_group.stop()

    # Place the robot to its home position to begin place movement
    joint_goal = meca_arm_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    meca_arm_group.go(joint_goal, wait=True)
    meca_arm_group.stop()

    # Rotate the robot 90%
    joint_goal = meca_arm_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = 1.5708
    meca_arm_group.go(joint_goal, wait=True)
    meca_arm_group.stop()

    # Perform the pre-place movement
    waypoints = []
    current_pose = meca_arm_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = meca_arm_group.get_current_pose()

    new_eef_pose = geometry_msgs.msg.Pose()

    new_eef_pose.position.y = current_pose.pose.position.y + 0.05
    new_eef_pose.position.z = current_pose.pose.position.z - 0.4
    new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

    waypoints.append(new_eef_pose)

    fraction = 0.0
    for count_cartesian_path in range(0, 3):
        if fraction < 1.0:
            (plan_cartesian, fraction) = meca_arm_group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,
                0.0)
        else:
            print('error')
            break

    meca_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    meca_arm_goal.trajectory = plan_cartesian
    meca_arm_client.send_goal(meca_arm_goal)
    meca_arm_client.wait_for_result()

    # move robot up in order to place cube
    joint_goal = meca_fingers_group.get_current_joint_values()
    joint_goal[0] = 0.040
    meca_fingers_group.go(joint_goal, wait=True)
    meca_fingers_group.stop()
    rospy.sleep(2)

    # move robot up a bit to clear the cube
    waypoints = []
    current_pose = meca_arm_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = meca_arm_group.get_current_pose()

    new_eef_pose = geometry_msgs.msg.Pose()

    new_eef_pose.position.y = current_pose.pose.position.y
    new_eef_pose.position.z = current_pose.pose.position.z + 0.4
    new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)

    waypoints.append(new_eef_pose)
    fraction = 0.0
    for count_cartesian_path in range(0, 3):
        if fraction < 1.0:
            (plan_cartesian, fraction) = meca_arm_group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,
                0.0)
        else:
            print('error')
            break

    meca_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    meca_arm_goal.trajectory = plan_cartesian
    meca_arm_client.send_goal(meca_arm_goal)
    meca_arm_client.wait_for_result()

    # Return to robot home position
    joint_goal = meca_arm_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    meca_arm_group.go(joint_goal, wait=True)
    meca_arm_group.stop()

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        mecademic_robot_basic_movement()
    except rospy.ROSInterruptException:
        pass
