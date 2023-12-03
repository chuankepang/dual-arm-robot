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
        arm1 = MoveGroupCommander('left_arm')
        arm2 = MoveGroupCommander('right_arm')

        # 当运动规划失败后，允许重新规划
        arm1.allow_replanning(True)
        arm2.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm1.set_pose_reference_frame('world')
        arm2.set_pose_reference_frame('world')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm1.set_goal_position_tolerance(0.001)
        arm1.set_goal_orientation_tolerance(0.001)
        arm2.set_goal_position_tolerance(0.001)
        arm2.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm1.set_max_acceleration_scaling_factor(0.05)
        arm1.set_max_velocity_scaling_factor(0.05)
        arm2.set_max_acceleration_scaling_factor(0.05)
        arm2.set_max_velocity_scaling_factor(0.05)
        
        # 获取终端link的名称
        end_effector_link1 = arm1.get_end_effector_link()
        end_effector_link2 = arm2.get_end_effector_link()
 

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose1 = arm1.get_current_pose(end_effector_link1).pose
        start_pose2 = arm2.get_current_pose(end_effector_link2).pose

        # 抓取
        left_dh3_open_publisher.left_gripper_open_publisher()
        #right_dh3_open_publisher.right_gripper_open_publisher()
        rospy.sleep(2)


        # 第一段 预抓取点
        joint1 = -5.6608784834491175
        joint2 = -1.601027627984518
        joint3 = -2.0397520065307617
        joint4 = 3.640335722560547
        joint5 = 5.332748889923096
        joint6 = 2.096283435821533

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm1.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))


                 
        # 控制机械臂完成运动
        arm1.go()


        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
