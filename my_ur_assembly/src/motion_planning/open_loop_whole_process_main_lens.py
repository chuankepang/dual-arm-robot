#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
import left_dh3_close_publisher
import left_dh3_open_publisher
import left_dh3_3fingers_publisher
import left_dh3_2fingers_publisher
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
		rospy.sleep(2)
		left_dh3_2fingers_publisher.left_gripper_2fingers_publisher()
		rospy.sleep(2)

		# 抓取
		#left_gripper_open_publisher()
		#rospy.sleep(2)

		# 第一段 预抓取点
		# 需要重新标定一下
		#[-5.36477500597109, -1.660098215142721, -2.475987434387207, 4.165722532863281, 5.628739356994629, 2.1105504035949707]
		joint1 = -5.36477500597109
		joint2 = -1.660098215142721
		joint3 = -2.475987434387207
		joint4 = 4.165722532863281
		joint5 = 5.628739356994629
		joint6 = 2.1105504035949707

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()
		rospy.sleep(1)



		#[-5.753651444111959, -2.031405111352438, -1.924006462097168, 3.970249815578125, 5.237756729125977, 2.097290515899658]
		joint1 = -5.753651444111959
		joint2 = -2.031405111352438
		joint3 = -1.924006462097168
		joint4 = 3.970249815578125
		joint5 = 5.237756729125977
		joint6 = 2.097290515899658

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()




		rospy.sleep(2)
		cartesian=1
		start_pose = arm.get_current_pose(end_effector_link).pose
		# 第二段走直线
		# 初始化路点列表
		waypoints1 = []

		# 如果为True,将初始位姿加入路点列表
		if cartesian:
			waypoints1.append(start_pose)
			
		# 设置路点数据，并加入路点列表，所有的点都加入
		wpose1 = deepcopy(start_pose)#拷贝对象
		wpose1.position.x += 0.046

		if cartesian:  #如果设置为True，那么走直线
			waypoints1.append(deepcopy(wpose1))
		else:          #否则就走曲线
			arm.set_pose_target(wpose1)  #自由曲线
			arm.go()
			rospy.sleep(1)


		#规划过程

		if cartesian:
			fraction = 0.0   #路径规划覆盖率
			maxtries = 100   #最大尝试规划次数
			attempts = 0     #已经尝试规划次数

		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
		while fraction < 1.0 and attempts < maxtries:
		#规划路径 ，fraction返回1代表规划成功
			(plan, fraction) = arm.compute_cartesian_path (
									waypoints1,   # waypoint poses，路点列表，这里是5个点
									0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
									0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
									True)        # avoid_collisions，避障规划
			# new_plan = scale_trajectory_speed(plan, 0.25)
			new_plan = arm.retime_trajectory(arm.get_current_state(), plan, 0.3)
			
			# 尝试次数累加
			attempts += 1
			
			# 打印运动规划进程
			if attempts % 10 == 0:
				rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
						
		# 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
		if fraction == 1.0:
			rospy.loginfo("Path computed successfully. Moving the arm.")
			arm.execute(new_plan)
			rospy.loginfo("Path execution complete.")
		# 如果路径规划失败，则打印失败信息
		else:
			rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

		rospy.sleep(2)


		left_dh3_close_publisher.left_gripper_close_publisher()
		rospy.sleep(2)

		cartesian=1
		start_pose = arm.get_current_pose(end_effector_link).pose
		# 第二段走直线
		# 初始化路点列表
		waypoints1 = []

		# 如果为True,将初始位姿加入路点列表
		if cartesian:
			waypoints1.append(start_pose)
			
		# 设置路点数据，并加入路点列表，所有的点都加入
		wpose1 = deepcopy(start_pose)#拷贝对象
		wpose1.position.x -= 0.31

		if cartesian:  #如果设置为True，那么走直线
			waypoints1.append(deepcopy(wpose1))
		else:          #否则就走曲线
			arm.set_pose_target(wpose1)  #自由曲线
			arm.go()
			rospy.sleep(1)


		#规划过程

		if cartesian:
			fraction = 0.0   #路径规划覆盖率
			maxtries = 100   #最大尝试规划次数
			attempts = 0     #已经尝试规划次数

		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
		while fraction < 1.0 and attempts < maxtries:
		#规划路径 ，fraction返回1代表规划成功
			(plan, fraction) = arm.compute_cartesian_path (
									waypoints1,   # waypoint poses，路点列表，这里是5个点
									0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
									0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
									True)        # avoid_collisions，避障规划
			# new_plan = scale_trajectory_speed(plan, 0.25)
			new_plan = arm.retime_trajectory(arm.get_current_state(), plan, 0.2)
			
			# 尝试次数累加
			attempts += 1
			
			# 打印运动规划进程
			if attempts % 10 == 0:
				rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
						
		# 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
		if fraction == 1.0:
			rospy.loginfo("Path computed successfully. Moving the arm.")
			arm.execute(new_plan)
			rospy.loginfo("Path execution complete.")
		# 如果路径规划失败，则打印失败信息
		else:
			rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

		rospy.sleep(2)

		# tfl.waitForTransform('world', 'left_ee_link', rospy.Time(0), rospy.Duration(4.0))
		# (trans,rot) = tfl.lookupTransform('world', 'left_ee_link', rospy.Time(0))

		# target_pose = PoseStamped()
		# #参考坐标系，前面设置了
		# target_pose.header.frame_id = reference_frame
		# target_pose.header.stamp = rospy.Time.now() #时间戳？

		# target_pose.pose.position.x = trans[0] - 0.30
		# target_pose.pose.position.y = trans[1]
		# target_pose.pose.position.z = trans[2]
		# target_pose.pose.orientation.x = rot[0]
		# target_pose.pose.orientation.y = rot[1]
		# target_pose.pose.orientation.z = rot[2]
		# target_pose.pose.orientation.w = rot[3]
		# arm.set_start_state_to_current_state()

		# arm.set_pose_target(target_pose, end_effector_link)
		# arm.go()
		# rospy.sleep(2)





		#[-5.938052002583639, -1.6886917553343714, -1.8683385848999023, 5.302002894669332, 5.2139363288879395, 4.3288893699646]
		joint1 = -5.938052002583639
		joint2 = -1.6886917553343714
		joint3 = -1.8683385848999023
		joint4 = 5.302002894669332
		joint5 = 5.2139363288879395
		joint6 = 4.3288893699646

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()
		rospy.sleep(2)

		cartesian=1
		start_pose = arm.get_current_pose(end_effector_link).pose
		# 第二段走直线
		# 初始化路点列表
		waypoints1 = []

		# 如果为True,将初始位姿加入路点列表
		if cartesian:
			waypoints1.append(start_pose)
			
		# 设置路点数据，并加入路点列表，所有的点都加入
		wpose1 = deepcopy(start_pose)#拷贝对象
		wpose1.position.y += 0.08

		if cartesian:  #如果设置为True，那么走直线
			waypoints1.append(deepcopy(wpose1))
		else:          #否则就走曲线
			arm.set_pose_target(wpose1)  #自由曲线
			arm.go()
			rospy.sleep(1)


		#规划过程

		if cartesian:
			fraction = 0.0   #路径规划覆盖率
			maxtries = 100   #最大尝试规划次数
			attempts = 0     #已经尝试规划次数

		# 设置机器臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
		while fraction < 1.0 and attempts < maxtries:
		#规划路径 ，fraction返回1代表规划成功
			(plan, fraction) = arm.compute_cartesian_path (
									waypoints1,   # waypoint poses，路点列表，这里是5个点
									0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
									0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
									True)        # avoid_collisions，避障规划
			# new_plan = scale_trajectory_speed(plan, 0.25)
			new_plan = arm.retime_trajectory(arm.get_current_state(), plan, 0.05)
			
			# 尝试次数累加
			attempts += 1
			
			# 打印运动规划进程
			if attempts % 10 == 0:
				rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
						
		# 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
		if fraction == 1.0:
			rospy.loginfo("Path computed successfully. Moving the arm.")
			arm.execute(plan)
			rospy.loginfo("Path execution complete.")
		# 如果路径规划失败，则打印失败信息
		else:
			rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

		rospy.sleep(2)

		left_dh3_open_publisher.left_gripper_open_publisher()
		rospy.sleep(2)

		# 第一段 预抓取点
		# 需要重新标定一下
		#[-5.36477500597109, -1.660098215142721, -2.475987434387207, 4.165722532863281, 5.628739356994629, 2.1105504035949707]
		joint1 = -5.36477500597109
		joint2 = -1.660098215142721
		joint3 = -2.475987434387207
		joint4 = 4.165722532863281
		joint5 = 5.628739356994629
		joint6 = 2.1105504035949707

		joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
		result=arm.set_joint_value_target(joint_positions)
		rospy.loginfo(str(result))


					
		# 控制机械臂完成运动
		arm.go()

		#moveit_commander.roscpp_shutdown()
		#moveit_commander.os._exit(0)

if __name__ == "__main__":
	try:
		MoveItCartesianDemo()
	except rospy.ROSInterruptException:
		pass
