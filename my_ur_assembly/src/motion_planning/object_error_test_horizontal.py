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
from left_dh3_open_publisher import left_gripper_open_publisher
from left_dh3_close_publisher import left_gripper_close_publisher
from right_dh3_open_publisher import right_gripper_open_publisher
from right_dh3_close_publisher import right_gripper_close_publisher
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
 
class MoveItCartesianDemo:
	def __init__(self):
		
		# 初始化move_group的API
		moveit_commander.roscpp_initialize(sys.argv)
		
		# 初始化ROS节点
		rospy.init_node('moveit_cartesian_demo', anonymous=True)

		# 初始化需要使用move group控制的机械臂中的arm group
		arm = MoveGroupCommander('left_arm')
		
		# 当运动规划失败后，允许重新规划
		arm.allow_replanning(True)
		
		# 设置目标位置所使用的参考坐标系
		reference_frame = 'world'
		arm.set_pose_reference_frame(reference_frame)
		end_effector_link = 'left_ee_link'

		# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
		arm.set_goal_position_tolerance(0.001)
		arm.set_goal_orientation_tolerance(0.001)

		# 设置允许的最大速度和加速度
		arm.set_max_acceleration_scaling_factor(0.5)
		arm.set_max_velocity_scaling_factor(0.5)

		# 获取终端link的名称
		end_effector_link = arm.get_end_effector_link()


		# 获取当前位姿数据最为机械臂运动的起始位姿
		start_pose = arm.get_current_pose(end_effector_link).pose

		print(end_effector_link)


		# 第一段 预抓取点
		# 需要重新标定一下
		joint1 = -6.154663864766256
		joint2 = -0.3350556653789063
		joint3 = -1.173274040222168
		joint4 = 5.969629752426901
		joint5 = 2.5760915279388428
		joint6 = -1.3509047667132776

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()



		rospy.sleep(1)


		# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
		# 姿态使用四元数描述，基于base_link坐标系
		target_pose = PoseStamped()
		#参考坐标系，前面设置了
		target_pose.header.frame_id = reference_frame
		target_pose.header.stamp = rospy.Time.now() #时间戳？

		#末端位置2，右手预备位置1
		#-0.692, 1.216, 0.817
		target_pose.pose.position.x = -0.7
		target_pose.pose.position.y = 1.2
		target_pose.pose.position.z = 0.7

		#0.620, 0.363, -0.597, 0.358
		target_pose.pose.orientation.x = -0.7071068 
		target_pose.pose.orientation.y = 0
		target_pose.pose.orientation.z = 0.7071068
		target_pose.pose.orientation.w = 0



		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose, end_effector_link)
		arm.go()


		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

if __name__ == "__main__":
	try:
		MoveItCartesianDemo()
	except rospy.ROSInterruptException:
		pass
