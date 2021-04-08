#!/usr/bin/env python
import os
os.environ["ROS_NAMESPACE"] = "/robot2"
import sys
import rospy
import moveit_commander
from MoveGroup import MoveGroup
from std_msgs.msg import String

def mecademic_robot_basic_movement():
    # First initialize moveit_commander and rospy to interact with the MoveIt node and facilitate communication within the ros env
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)


    # Instantiate a MoveGroupCommander object. 
    meca_arm_group = MoveGroup("meca_arm")

    # MoveGroup Commander Object for the mecademic hand.
    meca_fingers_group = MoveGroup("hand")

    # Ensure that the robot begins at its home position
    meca_arm_group.move_to_home()

    # Ensure that the robot fingers are opened to pick up cube
    meca_fingers_group.move_via_joint_values([0.040, 0.040])


    # rotate the robot 179 degrees
    # meca_arm_group.move_via_joint_values([-3.12414])


    #Cartesian path movement to pre grasp position
    meca_arm_group.relative_cartesian_movement([.029, -999, -.25])

    # Close the mecademic robot fingers to pick bin up.
    meca_fingers_group.move_via_joint_values([0.00, 0.00])

    # Lift the EEF upwards to ensure that the bin does not drag on the ground
    meca_arm_group.relative_cartesian_movement([.02, -999, .2])

    # Place the robot to its home position
    # meca_arm_group.relative_cartesian_movement([.1, -999, -.4])

    # Initiate topic to publish messages
    pub = rospy.Publisher('state_of_sequence1', String, queue_size=10)
    # Set rate at which the message will be published
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # publish message 
        pub.publish("1") 
        rate.sleep()
       
    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        print("running Program")
        mecademic_robot_basic_movement()
    except rospy.ROSInterruptException:
        print("Error occurred")
        pass
