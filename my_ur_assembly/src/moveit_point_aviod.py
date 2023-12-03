#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import thread, copy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy
sys.path.append("/home/ck/ur_ws/src/ur_moveit/src/GripperTestPython")
from GripperClose import socket_closegripper
sys.path.append("/home/ck/ur_ws/src/ur_moveit/src/GripperTestPython/GripperTestPython/GripperOpen")
from GripperOpen  import socket_opengripper
 
class MoveAttachedObjectDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_point_aviod')
        
        # 初始化场景对象，用来监听外部环境的变化
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')
        
        # 获取终端link的名称
        end_effector_link = 'tool0'

	# 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
       
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        arm.set_planning_time(10)
 
        # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        
        # 移除场景中之前运行残留的物体
        # scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('table') 
        scene.remove_world_object('target')
 
        # 设置桌面的高度
        table_ground = 0.6
        
        # 设置table和tool的三维尺寸
        table1_size = [0.5, 1.2, 0.6]  # 设置长宽高
	
        
        
 
        # 将table加入场景当中
        table1_pose = PoseStamped()
        table1_pose.header.frame_id = 'world'
        table1_pose.pose.position.x = 0.6
        table1_pose.pose.position.y = 0
        table1_pose.pose.position.z = 0.3
        table1_pose.pose.orientation.w = 1.0
        scene.add_box('table1', table1_pose, table1_size)  #添加障碍物
        
        rospy.sleep(2)  


	# 设置table和tool的三维尺寸
        # table2_size = [1.5, 0.01, 1.5]  # 设置长宽高
        
        
        
 
        # 将table加入场景当中
        #table2_pose = PoseStamped()
        #table2_pose.header.frame_id = 'base_link'
        #table2_pose.pose.position.x = 0.00
        #table2_pose.pose.position.y = -0.7
        #table2_pose.pose.position.z = 0.9
        #table2_pose.pose.orientation.w = 1.0
        #scene.add_box('table2', table2_pose, table2_size)  #添加障碍物
        
        #rospy.sleep(2)  



	# 设置table和tool的三维尺寸
        #table3_size = [2.5, 2.5, 0.01]  # 设置长宽高
        
        
        
 
        # 将table加入场景当中
        #table3_pose = PoseStamped()
        #table3_pose.header.frame_id = 'base_link'
        #table3_pose.pose.position.x = 0.00
        #table3_pose.pose.position.y = 0.00
        #table3_pose.pose.position.z = -0.05
        #table3_pose.pose.orientation.w = 1.0
        #scene.add_box('table3', table3_pose, table3_size)  #添加障碍物
        
        #rospy.sleep(2)  

	
 
        # 更新当前的位姿
        arm.set_start_state_to_current_state()
 
        

	# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        #参考坐标系，前面设置了
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置1   
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.5
        target_pose.pose.position.z = 0.9
        #末端姿态，四元数
        target_pose.pose.orientation.x = 1
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
	
	#socket_opengripper()

        rospy.sleep(5)  #执行完成后休息1s


	

	#末端位置2   
        target_pose.pose.position.x =  0.5
        target_pose.pose.position.y =  0.5
        target_pose.pose.position.z =  0.7
        #末端姿态，四元数
        target_pose.pose.orientation.x = 1
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0



	# 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
	
	#socket_closegripper()        

	rospy.sleep(5)  #执行完成后休息1s


	
	
	

	#末端位置3   
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.5
        target_pose.pose.position.z = 0.9
        #末端姿态，四元数
        target_pose.pose.orientation.x = 1
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(5)  #执行完成后休息1s


	##socket_closegripper()


	#末端位置4   
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = -0.5
        target_pose.pose.position.z = 0.9
        #末端姿态，四元数
        target_pose.pose.orientation.x = 1
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(5)  #执行完成后休息1s


	##socket_closegripper()


	#末端位置5   
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = -0.5
        target_pose.pose.position.z = 0.7
        #末端姿态，四元数
        target_pose.pose.orientation.x = 1
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)

	#socket_opengripper()


        rospy.sleep(5)  #执行完成后休息1s


	


	#末端位置6   
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = -0.5
        target_pose.pose.position.z = 0.9
        #末端姿态，四元数
        target_pose.pose.orientation.x = 1
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(5)  #执行完成后休息1s


	##socket_closegripper()


                 
        
        
        # 控制机械臂回到初始化位置
        #arm.set_named_target('home')
        #arm.go()
 
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    MoveAttachedObjectDemo()
