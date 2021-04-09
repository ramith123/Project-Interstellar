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
VLENGTH = 0.112
LENGTH = 0.104



if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('simple_pick_place', anonymous=True)
    pp = Pick_Place()# print(pp.get_object_list())
    pp.back_to_home()
    # rospy.sleep(30)
    pp = Pick_Place()
    
    
    pp.add_target_from_Objects()
    

    
    
    # # pp.move_pose_arm(grasp.grasp_pose.pose)
    # # rospy.sleep(2)

    def pickup_and_drop_seq(object_name,target_name,orientation_obj,orientation_tar):
        boxPose = pp.get_object_p(object_name)
        pp.scene.add_box(object_name, boxPose,(0.01,0.01,0.01))
        if (orientation_obj == "horizontal"):
            grasp = pp.generate_grasp(object_name, orientation_obj, boxPose.pose.position,pitch=30, length=VLENGTH)
        elif (orientation_obj == "vertical"):
            grasp = pp.generate_grasp(object_name, orientation_obj, boxPose.pose.position, length=VLENGTH)
        else:
            return
      
        
        pp.pickup(object_name, [grasp])
        pp.clean_scene(object_name)
        place_position = pp.get_target_position(target_name)
        # print(place_position)
        if (orientation_tar == "horizontal"):
            pp.place(orientation_tar, place_position,pitch = 30)
        elif (orientation_tar == "vertical"):
            pp.place(orientation_tar, place_position,yaw = -90)
        else:
            return

    rospy.sleep(2)
    pickup_and_drop_seq("box1","box1Target","horizontal","vertical")
    # pp.move_joint_hand(0)
    rospy.sleep(2)
    pp.back_to_home()
    
    pp = Pick_Place()
    pickup_and_drop_seq("box2","box2Target","horizontal","vertical")
    pp.back_to_home()
    pickup_and_drop_seq("box1","box1Bin","vertical","horizontal")
    pp.back_to_home()
    pickup_and_drop_seq("box2","box2Bin","vertical","horizontal")
    
    # rospy.sleep(2)