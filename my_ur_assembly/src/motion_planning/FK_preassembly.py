#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 预取位置
 
import rospy, sys
import moveit_commander
import math
 
class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
 
        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('left_arm')
        
        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        
        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        rospy.sleep(1)
         
        # 设置夹爪的目标位置，并控制夹爪运动
        '''
        gripper.set_joint_value_target([0.01])
        gripper.go()
        rospy.sleep(1)
        '''
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

        pose=arm.get_current_pose('left_tool0')
        print('pose=',pose)
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
