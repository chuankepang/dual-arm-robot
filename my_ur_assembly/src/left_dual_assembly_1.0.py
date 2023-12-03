#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import copy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy
""" sys.path.append("/home/ck/catkin_ws/src/ur_moveit/src/GripperTestPython")
from GripperClose import socket_closegripper
sys.path.append("/home/ck/catkin_ws/src/ur_moveit/src/GripperTestPython/GripperTestPython/GripperOpen")
from GripperOpen  import socket_opengripper """
from dh3_close_publisher_test import gripper_close_publisher
from dh3_open_publisher_test import gripper_open_publisher
import os
sys.path.append("/home/ck/catkin_ws/src/ur_moveit/src")


 
class MoveItIkDemo:
    def __init__(self):
 
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

	# 初始化场景对象，用来监听外部环境的变化
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm1 = moveit_commander.MoveGroupCommander('left_arm')
        

        # 获取终端link的名称，这个在setup assistant中设置过了
        end_effector_link1 = 'left_tool0'
        
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm1.set_pose_reference_frame(reference_frame)
        
                
        # 当运动规划失败后，允许重新规划
        arm1.allow_replanning(True)
       

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm1.set_goal_position_tolerance(0.03)
        arm1.set_goal_orientation_tolerance(0.03)
        

        # 设置允许的最大速度和加速度
        arm1.set_max_acceleration_scaling_factor(0.5)
        arm1.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置
        # arm1.set_named_target('home')
        # arm1.go()
        # arm2.set_named_target('home')
        # arm2.go()
        # rospy.sleep(1)


	# 移除场景中之前运行残留的物体
        #scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('table') 
        #scene.remove_world_object('target')




        # 设置table和tool的三维尺寸
        table1_size = [0.6, 1.2, 0.9]  # 设置长宽高
	
        
        
 
        # 将table加入场景当中
        table1_pose = PoseStamped()
        table1_pose.header.frame_id = 'world'
        table1_pose.pose.position.x = 1
        table1_pose.pose.position.y = 0
        table1_pose.pose.position.z = 0.45
        table1_pose.pose.orientation.w = 1.0
        scene.add_box('table1', table1_pose, table1_size)  #添加障碍物
        
        rospy.sleep(2)  
               



        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose1 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose1.header.frame_id = reference_frame
        target_pose1.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置4   
        target_pose1.pose.position.x = 1
        target_pose1.pose.position.y = 0.4
        target_pose1.pose.position.z = 1.4
        #末端姿态，四元数
        target_pose1.pose.orientation.x = -0.707
        target_pose1.pose.orientation.y = 0.707
        target_pose1.pose.orientation.z = 0
        target_pose1.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
       
        arm1.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
       
        arm1.set_pose_target(target_pose1, end_effector_link1)

        # 规划运动路径，返回虚影的效果
       
        plan_success, traj1, planning_time, error_code = arm1.plan()
        
        # 按照规划的运动路径控制机械臂运动
        
        
        arm1.execute(traj1)
	#socket_opengripper()
	#gripper_open_publisher()
        # 
        os.system("/home/ck/catkin_ws/src/ur_moveit/src/dh3_open_publisher_test.py")
        rospy.sleep(1)  #执行完成后休息1s




        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose2 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose2.header.frame_id = reference_frame
        target_pose2.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置4   
        target_pose2.pose.position.x = 1
        target_pose2.pose.position.y = 0.4
        target_pose2.pose.position.z = 1.1
        #末端姿态，四元数
        target_pose2.pose.orientation.x = -0.707
        target_pose2.pose.orientation.y = 0.707
        target_pose2.pose.orientation.z = 0
        target_pose2.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
       
        arm1.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
       
        arm1.set_pose_target(target_pose2, end_effector_link1)

        # 规划运动路径，返回虚影的效果
       
        plan_success, traj1, planning_time, error_code = arm1.plan()
        
        # 按照规划的运动路径控制机械臂运动
      
        arm1.execute(traj1)
	#socket_opengripper()
	#gripper_close_publisher()
        os.system("/home/ck/catkin_ws/src/ur_moveit/src/dh3_close_publisher_test.py")
        rospy.sleep(2)  #执行完成后休息1s




        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose3 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose3.header.frame_id = reference_frame
        target_pose3.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置4   
        target_pose3.pose.position.x = 1
        target_pose3.pose.position.y = 0.4
        target_pose3.pose.position.z = 1.4
        #末端姿态，四元数
        target_pose3.pose.orientation.x = -0.707
        target_pose3.pose.orientation.y = 0.707
        target_pose3.pose.orientation.z = 0
        target_pose3.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
       
        arm1.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
       
        arm1.set_pose_target(target_pose3, end_effector_link1)

        # 规划运动路径，返回虚影的效果
       
        plan_success, traj1, planning_time, error_code = arm1.plan()
        
        # 按照规划的运动路径控制机械臂运动
      
        arm1.execute(traj1)
	#socket_opengripper()
	#gripper_open_publisher()

        #rospy.sleep(2)  #执行完成后休息1s




        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose4 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose4.header.frame_id = reference_frame
        target_pose4.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置4   
        target_pose4.pose.position.x = 1
        target_pose4.pose.position.y = -0.4
        target_pose4.pose.position.z = 1.4
        #末端姿态，四元数
        target_pose4.pose.orientation.x = -0.707
        target_pose4.pose.orientation.y = 0.707
        target_pose4.pose.orientation.z = 0
        target_pose4.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
       
        arm1.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
       
        arm1.set_pose_target(target_pose4, end_effector_link1)

        # 规划运动路径，返回虚影的效果
       
        plan_success, traj1, planning_time, error_code = arm1.plan()
        
        # 按照规划的运动路径控制机械臂运动
      
        arm1.execute(traj1)
	#socket_opengripper()
	#gripper_open_publisher()

        #rospy.sleep(2)  #执行完成后休息1s




        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose5 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose5.header.frame_id = reference_frame
        target_pose5.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置4   
        target_pose5.pose.position.x = 1
        target_pose5.pose.position.y = -0.4
        target_pose5.pose.position.z = 1.1
        #末端姿态，四元数
        target_pose5.pose.orientation.x = -0.707
        target_pose5.pose.orientation.y = 0.707
        target_pose5.pose.orientation.z = 0
        target_pose5.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
       
        arm1.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
       
        arm1.set_pose_target(target_pose5, end_effector_link1)

        # 规划运动路径，返回虚影的效果
       
        plan_success, traj1, planning_time, error_code = arm1.plan()
        
        # 按照规划的运动路径控制机械臂运动
      
        arm1.execute(traj1)
	#socket_opengripper()
	#gripper_open_publisher()
        os.system("/home/ck/catkin_ws/src/ur_moveit/src/dh3_open_publisher_test.py")
        rospy.sleep(2)  #执行完成后休息1s





        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose6 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose6.header.frame_id = reference_frame
        target_pose6.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置4   
        target_pose6.pose.position.x = 1
        target_pose6.pose.position.y = -0.4
        target_pose6.pose.position.z = 1.4
        #末端姿态，四元数
        target_pose6.pose.orientation.x = -0.707
        target_pose6.pose.orientation.y = 0.707
        target_pose6.pose.orientation.z = 0
        target_pose6.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
       
        arm1.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
       
        arm1.set_pose_target(target_pose6, end_effector_link1)

        # 规划运动路径，返回虚影的效果
       
        plan_success, traj1, planning_time, error_code = arm1.plan()
        
        # 按照规划的运动路径控制机械臂运动
      
        arm1.execute(traj1)
	#socket_opengripper()
	#gripper_open_publisher()

        #rospy.sleep(2)  #执行完成后休息1s





        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose7 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose7.header.frame_id = reference_frame
        target_pose7.header.stamp = rospy.Time.now() #时间戳？
        
	#末端位置4   
        target_pose7.pose.position.x = 1
        target_pose7.pose.position.y = 0.4
        target_pose7.pose.position.z = 1.4
        #末端姿态，四元数
        target_pose7.pose.orientation.x = -0.707
        target_pose7.pose.orientation.y = 0.707
        target_pose7.pose.orientation.z = 0
        target_pose7.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
       
        arm1.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
       
        arm1.set_pose_target(target_pose7, end_effector_link1)

        # 规划运动路径，返回虚影的效果
       
        plan_success, traj1, planning_time, error_code = arm1.plan()
        
        # 按照规划的运动路径控制机械臂运动
      
        arm1.execute(traj1)
	#socket_opengripper()
	#gripper_open_publisher()

        #rospy.sleep(2)  #执行完成后休息1s





 
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    MoveItIkDemo()

