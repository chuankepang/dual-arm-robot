#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
import left_dh3_open_publisher
import left_dh3_close_publisher
import right_dh3_open_publisher
import right_dh3_close_publisher
import left_dh3_3fingers_publisher
import left_dh3_2fingers_publisher
from left_dh3_open_publisher import left_gripper_open_publisher
from left_dh3_close_publisher import left_gripper_close_publisher
from right_dh3_open_publisher import right_gripper_open_publisher
from right_dh3_close_publisher import right_gripper_close_publisher
from left_dh3_3fingers_publisher import left_gripper_3fingers_publisher
from left_dh3_2fingers_publisher import left_gripper_2fingers_publisher
import tf
import tf2_ros
import frame_grasp_with_ArUco_horizontal
import awl_grasp_with_ArUco_horizontal
from frame_grasp_with_ArUco_horizontal import MoveItCartesianDemo
from awl_grasp_with_ArUco_horizontal import MoveItCartesianDemo
import open_loop_whole_process_main_lens
import open_loop_whole_process_tool
import open_loop_whole_process_second_lens
import open_loop_whole_process_pole
from geometry_msgs.msg import PoseStamped, Pose


if __name__ == "__main__":
	try:
		open_loop_whole_process_main_lens.MoveItCartesianDemo()
		open_loop_whole_process_tool.MoveItCartesianDemo()
		open_loop_whole_process_second_lens.MoveItCartesianDemo()
		open_loop_whole_process_pole.MoveItCartesianDemo()
	except rospy.ROSInterruptException:
		pass
