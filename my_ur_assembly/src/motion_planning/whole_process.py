#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
from dh3_open_publisher_test import gripper_open_publisher
from dh3_close_publisher_test import gripper_close_publisher
import dh3_open_publisher_test
import dh3_close_publisher_test
 
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
        arm.set_pose_reference_frame('world')
                
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

	# 抓取
        gripper_open_publisher()
	    rospy.sleep(2)
                
        # 第一段 预抓取点
        joint1 = 0.31730651855
        joint2 = -1.833644052545
        joint3 = -1.46886539459
        joint4 = 3.318955107326
        joint5 = -1.2592394987
        joint6 = 2.1198773384

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))


                 
        # 控制机械臂完成运动
        arm.go()

	    # 抓取
        #dh3_close_publisher_test.gripper_close_publisher()
	    #rospy.sleep(2)


	    rospy.sleep(1)

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
        wpose1.position.x += 0.105
	    wpose1.position.z += 0.000
 
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



	# 抓取
        gripper_close_publisher()
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
        wpose1.position.x -= 0.105
	    wpose1.position.z += 0.000
 
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

        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint1 = 0.38016414642
        joint2 = -2.05125774959
        joint3 = -1.722903251647
        joint4 = 3.790724917
        joint5 = -1.20161849657
        joint6 = 2.121531963348

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))
                 
        # 控制机械臂完成运动
        arm.go()

	    rospy.sleep(1)

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
        wpose1.position.x += 0.105
	    wpose1.position.z += 0.000
 
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

        gripper_open_publisher()
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
        wpose1.position.x -= 0.105
	    wpose1.position.z += 0.000
 
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



        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
