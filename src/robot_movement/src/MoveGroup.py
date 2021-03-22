#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs


class MoveGroup():

    def __init__(self, move_group_name):
        # Instantiate a MoveGroupCommander object.  This object is an interface
        # to one group of joints.  In this case the group refers to the joints of
        # the meca_arm.
        self.meca_group = moveit_commander.MoveGroupCommander(move_group_name)

        # Action clients to the ExecuteTrajectory action server
        self.meca_client = actionlib.SimpleActionClient('execute_trajectory',
                                                        moveit_msgs.msg.ExecuteTrajectoryAction)

        # Ensure that the server is ready to receive request
        self.meca_client.wait_for_server()

    def print_robot_state(self):
        print("============ Printing robot state:")
        print(self.meca_group.get_current_state())
        print("")

    def print_robot_joints(self):
        print("============ Printing robot current joint goals:")
        current_joint_state = self.meca_group.get_current_joint_values()
        print(current_joint_state)
        print("")
        return current_joint_state

    def move_via_joint_values(self, new_joint_states=[]):
        current_joint_state = self.meca_group.get_current_joint_values()
        for i, current_joint_value in enumerate(new_joint_states):
            if current_joint_value == -1:
                continue
            current_joint_state[i] = current_joint_value

         # Move to the specified joint goal
        self.meca_group.go(current_joint_state, wait=True)

        # ensures that there is no residual movement
        self.meca_group.stop()

    def move_to_home(self):

        # Set a named joint configuration as the goal to plan for a move group.
        # Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
        self.meca_group.set_named_target("home pose")

        # Plan to the desired joint-space goal using the default planner (RRTConnect).
        meca_home = self.meca_group.plan()
        # Create a goal message object for the action server.
        meca_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # Update the trajectory in the goal message.
        meca_goal.trajectory = meca_home

        # Send the goal to the action server.
        self.meca_client.send_goal(meca_goal)
        self.meca_client.wait_for_result()

    def absolute_cartesian_movement(self, position_array=[], orientation_array=[]):

        # Cartesian path movement to pre grasp position
        waypoints = []
        # start with the current pose
        current_pose = self.meca_group.get_current_pose()
        rospy.sleep(0.5)
        current_pose = self.meca_group.get_current_pose()

        # create linear offsets to the current pose
        new_eef_pose = geometry_msgs.msg.Pose()

        # Manual offsets because we don't have the senser coordinated from the gazebo plugin to detect cubes yet.

        if position_array[0] != -999:
            new_eef_pose.position.x = position_array[0]  # 0.05
        if position_array[1] != -999:
            new_eef_pose.position.y = position_array[1]  # - 0.4
        if position_array[2] != -999:
            new_eef_pose.position.z = position_array[2]  # - 0.4

        if orientation_array == []:
            # Retain orientation of the current pose.
            new_eef_pose.orientation = copy.deepcopy(
                current_pose.pose.orientation)
        else:
            new_eef_pose.orientation.x = orientation_array[0]
            new_eef_pose.orientation.y = orientation_array[1]
            new_eef_pose.orientation.z = orientation_array[2]

        waypoints.append(new_eef_pose)
        print(new_eef_pose)

        # We want the cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in cartesian
        # translation.  We will specify the jump threshold as 0.0, effectively
        # disabling it.
        fraction = 0.0
        for count_cartesian_path in range(0, 3):
            if fraction < 1.0:
                (plan_cartesian, fraction) = self.meca_group.compute_cartesian_path(
                    waypoints,   # waypoints to follow
                    0.01,        # eef_step
                    0.0)         # jump_threshold
            else:
                print('error')
                break

        meca_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        meca_goal.trajectory = plan_cartesian
        self.meca_client.send_goal(meca_goal)
        self.meca_client.wait_for_result()

    def relative_cartesian_movement(self, position_array=[], orientation_array=[]):

        # Cartesian path movement to pre grasp position
        waypoints = []
        # start with the current pose
        current_pose = self.meca_group.get_current_pose()
        rospy.sleep(0.5)
        current_pose = self.meca_group.get_current_pose()

        # create linear offsets to the current pose
        new_eef_pose = geometry_msgs.msg.Pose()

        # Manual offsets because we don't have the senser coordinated from the gazebo plugin to detect cubes yet.

        if position_array[0] != -999:
            new_eef_pose.position.x = current_pose.pose.position.x + \
                position_array[0]  # 0.05
        if position_array[1] != -999:
            new_eef_pose.position.y = current_pose.pose.position.y + \
                position_array[1]  # - 0.4
        if position_array[2] != -999:
            new_eef_pose.position.z = current_pose.pose.position.z + \
                position_array[2]  # - 0.4

        if orientation_array == []:
            # Retain orientation of the current pose.
            new_eef_pose.orientation = copy.deepcopy(
                current_pose.pose.orientation)
        else:
            new_eef_pose.orientation.x = orientation_array[0]
            new_eef_pose.orientation.y = orientation_array[1]
            new_eef_pose.orientation.z = orientation_array[2]

        waypoints.append(new_eef_pose)
        print(new_eef_pose)

        # We want the cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in cartesian
        # translation.  We will specify the jump threshold as 0.0, effectively
        # disabling it.
        fraction = 0.0
        for count_cartesian_path in range(0, 3):
            if fraction < 1.0:
                (plan_cartesian, fraction) = self.meca_group.compute_cartesian_path(
                    waypoints,   # waypoints to follow
                    0.01,        # eef_step
                    0.0)         # jump_threshold
            else:
                print('error')
                break

        meca_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        meca_goal.trajectory = plan_cartesian
        self.meca_client.send_goal(meca_goal)
        self.meca_client.wait_for_result()

    def go_to_pose_goal(self, position_array=[], orientation=[]):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = position_array[0]
        pose_goal.position.y = position_array[1]
        pose_goal.position.z = position_array[2]

        if orientation != []:
            pose_goal.orientation.x = orientation[0]
            pose_goal.orientation.y = orientation[1]
            pose_goal.orientation.z = orientation[2]

        self.meca_group.set_pose_target(pose_goal)

        plan = self.meca_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.meca_group.stop()
        self.meca_group.clear_pose_targets()
