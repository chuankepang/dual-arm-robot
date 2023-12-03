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
import left_dh3_3fingers_publisher
import left_dh3_2fingers_publisher
from left_dh3_3fingers_publisher import left_gripper_3fingers_publisher
from left_dh3_2fingers_publisher import left_gripper_2fingers_publisher
import tf
import math
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from scipy.spatial.transform import Rotation as R

class MoveItCartesianDemo:
	def __init__(self):
		# 柱状物放置位置：
		# Joints:[-5.923715893422262, -2.3508078060545863, -1.870316505432129, 5.251549708634176, 3.7690365314483643, 2.48154878616333]
		# [0.824, 0.191, 0.905] [-0.707, 0.000, 0.707, 0.000]
        # 柱状物预放置位置：
		# joints:[-6.058680836354391, -2.470286031762594, -1.879622459411621, 5.560282218247213, 3.711153745651245, 2.699796199798584]
		# [0.824, 0.191, 1.036]

		# home:
		# [-6.1846492926227015, -2.35605587581777, -2.194159507751465, 5.958368766098776, 3.6805622577667236, 2.9327094554901123]

		# 初始化move_group的API
		moveit_commander.roscpp_initialize(sys.argv)
		
		# 初始化ROS节点
		try:
			rospy.init_node('moveit_cartesian_demo', anonymous=True)
		except rospy.exceptions.ROSException as e:
			print("Node has already been initialized, do nothing")
		

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
		arm.set_max_acceleration_scaling_factor(0.1)
		arm.set_max_velocity_scaling_factor(0.1)

		# 获取终端link的名称
		end_effector_link = arm.get_end_effector_link()


		# 获取当前位姿数据最为机械臂运动的起始位姿
		start_pose = arm.get_current_pose(end_effector_link).pose

		print(end_effector_link)

		
		left_dh3_open_publisher.left_gripper_open_publisher()
		rospy.sleep(1)
		left_dh3_2fingers_publisher.left_gripper_2fingers_publisher()
		rospy.sleep(1)

		# 第一段 home
		# 需要重新标定一下
		joint1 = -6.2069881598102015
		joint2 = -2.475522657433981
		joint3 = -1.7808313369750977
		joint4 = 5.700444209366598
		joint5 = 3.6746714115142822
		joint6 = 2.974576711654663

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()


		rospy.sleep(5)

		tfl = tf.TransformListener()
		#第1个码的返回读数
		tfl.waitForTransform('world', '819_marker_frame', rospy.Time(0), rospy.Duration(4.0))
		(trans1,rot1) = tfl.lookupTransform('world', '819_marker_frame', rospy.Time(0))
		#第二个码的返回读数
		tfl.waitForTransform('world', '727_marker_frame', rospy.Time(0), rospy.Duration(4.0))
		(trans2,rot2) = tfl.lookupTransform('world', '727_marker_frame', rospy.Time(0))

		#trans = [0.5, 0.7, 0.5]
		#rot = [1, 0, 0, 0]
		#print(trans, rot)
		positionx = 0
		positiony = 0
		x1 = trans1[0]
		y1 = trans1[1]
		x2 = trans2[0]
		y2 = trans2[1]
		l = 0.183
		# a为带正负号的弧度值，从台面上方看，顺时针为正
		if(np.abs(y2-y1)>l):
			a = math.asin(1)
		else:
			a = math.asin(np.abs(y2-y1) / l)
		#angle =  np.pi / 2 - a

		# if(x1 >= x2):
		# 	# 上半平面
		# 	a = math.asin((y2-y1) / l)
		# 	if(y1 >= y2):
		# 		# 从台面上方看，逆时针旋转
		# 		angle = 
		# 	else:
		# 		angle = 
		# else:
		# 	a = -math.asin((y2-y1) / l) - np.pi
		# 	if(y1 >= y2):
		# 		angle = 
		# 	else:
		# 		angle = 

		if(x1 <= x2):
			if(y1 >= y2):
				angle = np.pi/2 - a
			else:
				angle = -np.pi/2 + a
		else:
			# 上半平面
			if(y1 >= y2):
				# 从台面上方看，逆时针旋转
				angle = -np.pi/2 + a
			else:
				angle = np.pi/2 - a

		
		print("angle", angle / np.pi * 180 , a / np.pi * 180)
		#R1 = np.array([[0, np.cos(np.pi+angle), np.cos(np.pi / 2 - angle)], [0, np.cos(angle), np.cos(angle)],[-1, 0, 0]])
		#Q = R2Q(R1)
		#rospy.sleep(2)
		Q = [-math.sqrt(2)/2 * np.cos(angle / 2), -math.sqrt(2)/2 * np.sin(angle / 2), math.sqrt(2)/2 * np.cos(angle / 2), -math.sqrt(2)/2 * np.sin(angle / 2)]
		#Q = Q / np.linalg.norm(Q)
		# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
		# 姿态使用四元数描述，基于base_link坐标系
		target_pose = PoseStamped()
		#参考坐标系，前面设置了
		target_pose.header.frame_id = reference_frame
		target_pose.header.stamp = rospy.Time.now() #时间戳？
		print("ROT", rot1)
		# R_qr_g = np.array([[0, -0.707, -0.707], [0, -0.707, 0.707],[-1, 0, 0]])
		# Q = R.from_matrix(R_qr_g).as_quat()
		# print("CZL", Q)
		#末端位置2，右手预备位置1   
		target_pose.pose.position.x = trans1[0]-0.15
		target_pose.pose.position.y = trans1[1]-0.02
		target_pose.pose.position.z = trans1[2]+0.20
		target_pose.pose.orientation.x = -0.70710678
		target_pose.pose.orientation.y = 0
		target_pose.pose.orientation.z = 0.70710678
		target_pose.pose.orientation.w = 0
		#[1, 0, 0, 0]是指向墙面[-0.707, 0, 0.707, 0]
		#print("ROT", rot)
		#rot[0] = -rot[0]
		#rot[1] = -rot[1]
		#rot[2] = -rot[2]
		#rot[3] = -rot[3]
		#R_qr_w = R.from_quat(rot).as_matrix()
		#R_qr_g = np.array([[0, 0, 1], [0, 1, 0],[-1, 0, 0]])
		#R_g_w = np.dot((R_qr_w), R_qr_g)
		#Q = R.from_matrix(R_g_w).as_quat()
		#target_pose.pose.orientation.x = Q[0]
		#target_pose.pose.orientation.y = Q[1]
		#target_pose.pose.orientation.z = Q[2]
		#target_pose.pose.orientation.w = Q[3]
		# print(Q)


		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose, end_effector_link)
		arm.go()
        

		rospy.sleep(2)
		# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
		# 姿态使用四元数描述，基于base_link坐标系
		target_pose = PoseStamped()
		#参考坐标系，前面设置了
		target_pose.header.frame_id = reference_frame
		target_pose.header.stamp = rospy.Time.now() #时间戳？

		#末端位置2，右手预备位置1   
		target_pose.pose.position.x = trans1[0]-0.15
		target_pose.pose.position.y = trans1[1]-0.02
		target_pose.pose.position.z = trans1[2]+0.03
		target_pose.pose.orientation.x = -0.70710678
		target_pose.pose.orientation.y = 0
		target_pose.pose.orientation.z = 0.70710678
		target_pose.pose.orientation.w = 0
		#target_pose.pose.orientation.x = Q[0]
		#target_pose.pose.orientation.y = Q[1]
		#target_pose.pose.orientation.z = Q[2]
		#target_pose.pose.orientation.w = Q[3] 
		# target_pose.pose.orientation.x = 0.70710678*rot[0] + 0.70710678*rot[2]
		# target_pose.pose.orientation.y = 0.70710678*rot[1] - 0.70710678*rot[3]
		# target_pose.pose.orientation.z = 0.70710678*rot[2] + 0.70710678*rot[0]
		# target_pose.pose.orientation.w = 0.70710678*rot[3] + 0.70710678*rot[1]




		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose, end_effector_link)
		arm.go()
		rospy.sleep(10)

		tfl = tf.TransformListener()

		tfl.waitForTransform('world', '819_marker_frame', rospy.Time(0), rospy.Duration(4.0))
		(trans,rot) = tfl.lookupTransform('world', '819_marker_frame', rospy.Time(0))

		print(trans, rot)



		rospy.sleep(2)

		if(x1 <= x2):
			if(y1 >= y2):
				positionx = trans[0] + 0.09*np.cos(a) - 0.06*np.sin(a)
				positiony = trans[1] - 0.09*np.sin(a) - 0.06*np.cos(a)
			else:
				positionx = trans[0] + 0.09*np.cos(a) - 0.06*np.sin(a)
				positiony = trans[1] + 0.09*np.sin(a) + 0.06*np.cos(a)
		else:
			# 上半平面
			if(y1 >= y2):
				# 从台面上方看，逆时针旋转
				positionx = trans[0] - 0.09*np.cos(a) - 0.06*np.sin(a)
				positiony = trans[1] - 0.09*np.sin(a) + 0.06*np.cos(a)
			else:
				positionx = trans[0] - 0.09*np.cos(a) - 0.06*np.sin(a)
				positiony = trans[1] + 0.09*np.sin(a) - 0.06*np.cos(a)


		print(positionx, positiony)
		
		target_pose = PoseStamped()
		#参考坐标系，前面设置了
		target_pose.header.frame_id = reference_frame
		target_pose.header.stamp = rospy.Time.now() #时间戳？

		#末端位置2，右手预备位置1   


				

		#target_pose.pose.position.x = trans[0]
		#target_pose.pose.position.y = trans[1]
		#target_pose.pose.position.z = trans[2] + 0.01
		target_pose.pose.position.x = positionx
		target_pose.pose.position.y = positiony
		target_pose.pose.position.z = trans[2] + 0.1
		#print("czlll", Q)
		#[1, 0, 0, 0]是指向墙面[-0.707, 0, 0.707, 0]是指向桌面
		target_pose.pose.orientation.x = Q[0]
		target_pose.pose.orientation.y = Q[1]
		target_pose.pose.orientation.z = Q[2]
		target_pose.pose.orientation.w = Q[3]




		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose, end_effector_link)
		arm.go()
		rospy.sleep(2)
		# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
		# 姿态使用四元数描述，基0.于base_link坐标系
		target_pose = PoseStamped()
		#参考坐标系，前面设置了
		target_pose.header.frame_id = reference_frame
		target_pose.header.stamp = rospy.Time.now() #时间戳？

		#末端位置2，右手预备位置1   
		
		#target_pose.pose.position.x = trans[0]
		#target_pose.pose.position.y = trans[1]
		#target_pose.pose.position.z = trans[2] + 0.01
		target_pose.pose.position.x = positionx
		target_pose.pose.position.y = positiony
		target_pose.pose.position.z = trans[2] - 0.035
		#print("czlll", Q)
		#[1, 0, 0, 0]是指向墙面[-0.707, 0, 0.707, 0]是指向桌面
		target_pose.pose.orientation.x = Q[0]
		target_pose.pose.orientation.y = Q[1]
		target_pose.pose.orientation.z = Q[2]
		target_pose.pose.orientation.w = Q[3]




		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose, end_effector_link)
		arm.go()
        
		rospy.sleep(2)

		left_dh3_close_publisher.left_gripper_close_publisher()

		rospy.sleep(1)


		target_pose = PoseStamped()
		#参考坐标系，前面设置了
		target_pose.header.frame_id = reference_frame
		target_pose.header.stamp = rospy.Time.now() #时间戳？

		#末端位置2，右手预备位置1  
 
		target_pose.pose.position.x = positionx
		target_pose.pose.position.y = positiony
		target_pose.pose.position.z = trans[2] + 0.25

		#[1, 0, 0, 0]是指向墙面[-0.707, 0, 0.707, 0]
		target_pose.pose.orientation.x = -0.70710678
		target_pose.pose.orientation.y = 0
		target_pose.pose.orientation.z = 0.70710678
		target_pose.pose.orientation.w = 0



		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose, end_effector_link)
		arm.go()

		rospy.sleep(1)
		#[-5.875746552144186, -2.859180589715475, -1.3592596054077148, 5.197098243027487, 3.7951831817626953, 2.4447035789489746]
		joint1 = -5.875746552144186
		joint2 = -2.859180589715475
		joint3 = -1.3592596054077148
		joint4 = 5.197098243027487
		joint5 = 3.7951831817626953
		joint6 = 2.4447035789489746

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()

		rospy.sleep(2)
		tfl.waitForTransform('world', 'left_ee_link', rospy.Time(0), rospy.Duration(4.0))
		(trans,rot) = tfl.lookupTransform('world', 'left_ee_link', rospy.Time(0))

		target_pose.pose.position.x = trans[0]
		target_pose.pose.position.y = trans[1] - 0.005
		target_pose.pose.position.z = trans[2] - 0.09
		target_pose.pose.orientation.x = rot[0]
		target_pose.pose.orientation.y = rot[1]
		target_pose.pose.orientation.z = rot[2]
		target_pose.pose.orientation.w = rot[3]
		arm.set_start_state_to_current_state()

		arm.set_pose_target(target_pose, end_effector_link)
		arm.go()
		rospy.sleep(4)

		# cartesian = 1
		# # 初始化路点列表
		# waypoints = []

		# # 如果为True,将初始位姿加入路点列表
		# if cartesian:
		# 	waypoints.append(start_pose)
			
		# # 设置路点数据，并加入路点列表，所有的点都加入
		# wpose = deepcopy(start_pose)#拷贝对象
		# wpose.position.z -= 0.111

		# if cartesian:  #如果设置为True，那么走直线
		# 	waypoints.append(deepcopy(wpose))
		# else:          #否则就走曲线
		# 	arm.set_pose_target(wpose)  #自由曲线
		# 	arm.go()
		# 	rospy.sleep(1)

		# #规划过程

		# if cartesian:
		# 	fraction = 0.0   #路径规划覆盖率
		# 	maxtries = 100   #最大尝试规划次数
		# 	attempts = 0     #已经尝试规划次数
			
		# # 设置机器臂当前的状态作为运动初始状态
		# arm.set_start_state_to_current_state()

		# 	# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
		# while fraction < 1.0 and attempts < maxtries:
		# #规划路径 ，fraction返回1代表规划成功
		# 	(plan, fraction) = arm.compute_cartesian_path (
		# 							waypoints,   # waypoint poses，路点列表，这里是5个点
		# 							0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
		# 							0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
		# 							True)        # avoid_collisions，避障规划
		# 	new_plan = arm.retime_trajectory(arm.get_current_state(), plan, 0.07)
		# 	# 尝试次数累加
		# 	attempts += 1
			
		# 	# 打印运动规划进程
		# 	if attempts % 10 == 0:
		# 		rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
						
		# # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
		# if fraction == 1.0:
		# 	rospy.loginfo("Path computed successfully. Moving the arm.")
		# 	arm.execute(new_plan)
		# 	rospy.loginfo("Path execution complete.")
		# # 如果路径规划失败，则打印失败信息
		# else:
		# 	rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

		# rospy.sleep(2)

		left_dh3_open_publisher.left_gripper_open_publisher()

		rospy.sleep(2)
		# 第一段 home
		# 需要重新标定一下
		joint1 = -6.2069881598102015
		joint2 = -2.475522657433981
		joint3 = -1.7808313369750977
		joint4 = 5.700444209366598
		joint5 = 3.6746714115142822
		joint6 = 2.974576711654663

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()
		rospy.sleep(2)

		#moveit_commander.roscpp_shutdown()
		#moveit_commander.os._exit(0)
		

if __name__ == "__main__":
	try:
		MoveItCartesianDemo()
	except rospy.ROSInterruptException:
		pass
