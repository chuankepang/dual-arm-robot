#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys

import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy
#sys.path.append("/home/ck/ur_ws/src/ur_moveit/src/GripperTestPython")
#from GripperClose import socket_closegripper
#sys.path.append("/home/ck/ur_ws/src/ur_moveit/src/GripperTestPython/GripperTestPython/GripperOpen")
#from GripperOpen  import socket_opengripper
#from dh3_close_publisher import gripper_close_publisher
#from dh3_open_publisher import gripper_open_publisher


x = 1
y = 0.3
z = 1.2
ox = -0.707
oy = 0.707
oz = 0
zw = 0  

# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)               

# 初始化场景对象，用来监听外部环境的变化
scene = PlanningSceneInterface()
#rospy.sleep(1)
# 初始化需要使用move group控制的机械臂中的arm group
arm1 = moveit_commander.MoveGroupCommander('left_arm')
arm2 = moveit_commander.MoveGroupCommander('right_arm') 
# 初始化需要使用move group控制的机械臂中的gripper group
#gripper = moveit_commander.MoveGroupCommander('gripper')       
# 获取终端link的名称，这个在setup assistant中设置过了
end_effector_link1 = 'left_tool0'
end_effector_link2 = 'right_tool0'                     
# 设置目标位置所使用的参考坐标系
reference_frame = 'world'
arm1.set_pose_reference_frame(reference_frame)
arm2.set_pose_reference_frame(reference_frame)
                
# 当运动规划失败后，允许重新规划
arm1.allow_replanning(True)
arm2.allow_replanning(True)

# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
arm1.set_goal_position_tolerance(0.03)
arm1.set_goal_orientation_tolerance(0.03)
arm2.set_goal_position_tolerance(0.03)
arm2.set_goal_orientation_tolerance(0.03)

# 设置允许的最大速度和加速度
arm1.set_max_acceleration_scaling_factor(0.5)
arm1.set_max_velocity_scaling_factor(0.5)
arm2.set_max_acceleration_scaling_factor(0.5)
arm2.set_max_velocity_scaling_factor(0.5)
# 控制机械臂先回到初始化位置
#arm.set_named_target('home')
#arm.go()
# 设置机器臂当前的状态作为运动初始状态
arm1.set_start_state_to_current_state()
target_pose = PoseStamped()
a = 1 
def xy_subscriber():
        rospy.init_node('xypose_subscriber', anonymous=True)
        rospy.Subscriber("chatter", Pose, xyCallback)
        rospy.spin()
def xyCallback(data):
        x = data.position.x
        y = data.position.y
        z = data.position.z
        ox = data.orientation.x
        oy = data.orientation.y
        oz = data.orientation.z
        ow = data.orientation.w
        
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x =  x*0.000625+0.7
        target_pose.pose.position.y = y*0.00083+0.1
        target_pose.pose.position.z = 1
        target_pose.pose.orientation.x = 0.707
        target_pose.pose.orientation.y =-0.707
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0
        print(target_pose)
        # 设置机械臂终端运动的目标位姿
        arm1.set_pose_target(target_pose, end_effector_link1)
        arm1.go()
        global a
        a+=1
        print(" count ",a) 
        print("\n position \n x : %f \n y : %f \n z : %f \n orientation \n x : %f \n y : %f \n z : %f \n w : %f", data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
     # 关闭并退出moveit
        #moveit_commander.roscpp_shutdown()
        #moveit_commander.os._exit(0)
        arm1.clear_pose_targets()
        print("清除") 
if __name__ == "__main__":
    xy_subscriber()
