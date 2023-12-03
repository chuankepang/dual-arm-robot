#!/usr/bin/env python3


from select_ik_solution import select_ik_solution
from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
import numpy as np
from Inverse_Kinematics import IK
 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
 
def move():
          #goal就是我们向发送的关节运动数据，实例化为FollowJointTrajectoryGoal()类
          goal = FollowJointTrajectoryGoal()
 
          #goal当中的trajectory就是我们要操作的，其余的Header之类的不用管
          goal.trajectory = JointTrajectory()
          
          goal.trajectory.joint_names = JOINT_NAMES
 
          #从joint_state话题上获取当前的关节角度值，因为后续要移动关节时要第一个值为参考
          joint_states = rospy.wait_for_message("joint_states",JointState)
          joints_pos = joint_states.position
 
          #tips：
          joints_pos_list = list(joints_pos)
          record = joints_pos_list[2]
          joints_pos_list[2] = joints_pos_list[0]
          joints_pos_list[0] = record
 
          #设定目标姿态T，该T是事先通过正运动学模块进行求解，并通过逆运动学进行验证求解，确保逆运动学有解
          x= -722.6
          T = [[-1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, -109.15000000000002], [0.0, 0.0, -1.0, 7.159000000000006], [0.0, 0.0, 0.0, 1.0]]
          joint_position = [0]*20
          solution = IK(T)
          joint_position[0] = select_ik_solution(joints_pos_list, solution)
 
          for i in range(1,20):
                    #在笛卡尔空间下设置20个点，每个点只在x方向移动10，其余方向维持不变
                    x = x+10
                    T = [[-1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, -109.15000000000002], [0.0, 0.0, -1.0, 7.159000000000006], [0.0, 0.0, 0.0, 1.0]]
                    #逆运动学求解每一个点的关节角度值，每一次求解均有8组解
                    solution = IK(T)
                    #选出与上一次求解结果最接近的解
                    joint_position[i] = select_ik_solution(joint_position[i-1], solution)
          
          time = 0.3
          goal.trajectory.points=[0]*20
          goal.trajectory.points[0]=JointTrajectoryPoint(positions=joints_pos,time_from_start=rospy.Duration(0))
          for i in range(1,20):
                    #将所有解赋给goal.trajectory.points
                    goal.trajectory.points[i]=JointTrajectoryPoint(positions=joint_position[i],time_from_start=rospy.Duration(time))
                    time = time+0.3
          client.send_goal(goal)
 
def pub_test():
          global client
 
          #初始化ros节点
          rospy.init_node("pub_action_test")
 
          #实例化一个action的类，命名为client，与上述client对应，话题为arm_controller/follow_joint_trajectory，消息类型为FollowJointTrajectoryAction
          client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
          print("Waiting for server...")
          #等待server
          client.wait_for_server()
          print("Connect to server")
          #执行move函数，发布action
          move()
 
if __name__ == "__main__":
          pub_test()
