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
 
class MoveAttachedObjectDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_attached_object_demo')
        
        # 初始化场景对象，用来监听外部环境的变化
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

	# 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
       
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        arm.set_planning_time(10)
 
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        
        # 移除场景中之前运行残留的物体
        scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('table') 
        scene.remove_world_object('target')
 
        # 设置桌面的高度
        table_ground = 0.6
        
        # 设置table和tool的三维尺寸
        table_size = [0.1, 0.7, 0.01]  # 设置长宽高
        tool_size = [0.2, 0.02, 0.02]
        
        # 设置tool的位姿
        p = PoseStamped() 
        #tool放置位置的参考坐标系
        p.header.frame_id = end_effector_link
        
        #tool的具体位置姿态
        p.pose.position.x = tool_size[0] / 2.0 - 0.025
        p.pose.position.y = 0
        p.pose.position.z = 0.0
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1


        # 将tool附着到机器人的终端，该函数为附着某物体到机器人上
        scene.attach_box(end_effector_link, 'tool', p, tool_size)
 
        # 将table加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 0.25
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box('table', table_pose, table_size)  #添加障碍物
        
        rospy.sleep(2)  
 
        # 更新当前的位姿
        arm.set_start_state_to_current_state()
 
        

	# 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        #参考坐标系，前面设置了
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now() #时间戳？
        #末端位置   
        target_pose.pose.position.x = 0.393
        target_pose.pose.position.y = 0.236
        target_pose.pose.position.z = 0.4787
        #末端姿态，四元数
        target_pose.pose.orientation.x = 0.70692
        target_pose.pose.orientation.y = 0.6
        target_pose.pose.orientation.z = 0.2
        target_pose.pose.orientation.w = 0.70729
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)  #执行完成后休息1s


                 
        
        
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
 
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    MoveAttachedObjectDemo()
