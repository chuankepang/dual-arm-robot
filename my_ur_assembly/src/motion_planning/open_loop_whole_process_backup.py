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
        right_dh3_open_publisher.right_gripper_open_publisher()
        rospy.sleep(2)
                
        # 第一段 预抓取点
        joint1 = -5.519986931
        joint2 = -2.370320936
        joint3 = -1.817734718
        joint4 = 4.0176216798
        joint5 = 5.4772353172
        joint6 = 2.2561116218

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm1.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))


                 
        # 控制机械臂完成运动
        arm1.go()

        rospy.sleep(2)

        # 第一段 预抓取点
        joint1 = -0.573557202016012
        joint2 = -1.1032846730998536
        joint3 = 2.5172274748431605
        joint4 = 1.731137676829956
        joint5 = -0.9952595869647425
        joint6 = 1.0052309036254883

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm2.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))


                 
        # 控制机械臂完成运动
        arm2.go()        

        rospy.sleep(2)
        

        cartesian=1
        start_pose = arm2.get_current_pose(end_effector_link2).pose
        # 第二段走直线
        # 初始化路点列表
        waypoints1 = []
 
        # 如果为True,将初始位姿加入路点列表
        if cartesian:
            waypoints1.append(start_pose)
            
        # 设置路点数据，并加入路点列表，所有的点都加入
        wpose1 = deepcopy(start_pose)#拷贝对象
        wpose1.position.x += 0.26
        wpose1.position.x += 0.0
 
        if cartesian:  #如果设置为True，那么走直线
            waypoints1.append(deepcopy(wpose1))
        else:          #否则就走曲线
            arm2.set_pose_target(wpose1)  #自由曲线
            arm2.go()
            rospy.sleep(1)


        #规划过程
 
        if cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm2.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
        #规划路径 ，fraction返回1代表规划成功
            (plan, fraction) = arm2.compute_cartesian_path (
                                    waypoints1,   # waypoint poses，路点列表，这里是5个点
                                    0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
                                    0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
                                    True)        # avoid_collisions，避障规划
            # new_plan = scale_trajectory_speed(plan, 0.25)
            new_plan = arm2.retime_trajectory(arm2.get_current_state(), plan, 0.05)
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                        
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm2.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(2)

        # 抓取
        right_dh3_close_publisher.right_gripper_close_publisher()
        rospy.sleep(2)

        cartesian=1
        start_pose = arm2.get_current_pose(end_effector_link2).pose
        # 第二段走直线
        # 初始化路点列表
        waypoints1 = []
 
        # 如果为True,将初始位姿加入路点列表
        if cartesian:
            waypoints1.append(start_pose)
            
        # 设置路点数据，并加入路点列表，所有的点都加入
        wpose1 = deepcopy(start_pose)#拷贝对象
        wpose1.position.z += 0.15
 
        if cartesian:  #如果设置为True，那么走直线
            waypoints1.append(deepcopy(wpose1))
        else:          #否则就走曲线
            arm2.set_pose_target(wpose1)  #自由曲线
            arm2.go()
            rospy.sleep(1)

        #规划过程
 
        if cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm2.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
        #规划路径 ，fraction返回1代表规划成功
            (plan, fraction) = arm2.compute_cartesian_path (
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
            arm2.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(2)

        # 第一段 预抓取点
        joint1 = -0.1668184439288538
        joint2 = -1.8418318233885707
        joint3 = 2.1659396330462855
        joint4 = 2.784614248866699
        joint5 = -1.4231422583209437
        joint6 = 1.0633759498596191

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm2.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))


                 
        # 控制机械臂完成运动
        arm2.go()

        cartesian=1
        start_pose = arm1.get_current_pose(end_effector_link1).pose
        # 第二段走直线
        # 初始化路点列表
        waypoints1 = []
 
        # 如果为True,将初始位姿加入路点列表
        if cartesian:
            waypoints1.append(start_pose)
            
        # 设置路点数据，并加入路点列表，所有的点都加入
        wpose1 = deepcopy(start_pose)#拷贝对象
        wpose1.position.x += 0.19
 
        if cartesian:  #如果设置为True，那么走直线
            waypoints1.append(deepcopy(wpose1))
        else:          #否则就走曲线
            arm1.set_pose_target(wpose1)  #自由曲线
            arm1.go()
            rospy.sleep(1)

        #规划过程
 
        if cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm1.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
        #规划路径 ，fraction返回1代表规划成功
            (plan, fraction) = arm1.compute_cartesian_path (
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
            arm1.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(2)

        left_dh3_close_publisher.left_gripper_close_publisher()
        rospy.sleep(2)


        cartesian=1
        start_pose = arm1.get_current_pose(end_effector_link1).pose
        # 第二段走直线
        # 初始化路点列表
        waypoints1 = []
 
        # 如果为True,将初始位姿加入路点列表
        if cartesian:
            waypoints1.append(start_pose)
            
        # 设置路点数据，并加入路点列表，所有的点都加入
        wpose1 = deepcopy(start_pose)#拷贝对象
        wpose1.position.z += 0.15
 
        if cartesian:  #如果设置为True，那么走直线
            waypoints1.append(deepcopy(wpose1))
        else:          #否则就走曲线
            arm1.set_pose_target(wpose1)  #自由曲线
            arm1.go()
            rospy.sleep(1)

        #规划过程
 
        if cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm1.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
        #规划路径 ，fraction返回1代表规划成功
            (plan, fraction) = arm1.compute_cartesian_path (
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
            arm1.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(2)

        # 第一段 预抓取点
        joint1 = -5.959046665822164
        joint2 = -2.670809408227438
        joint3 = -1.5416135787963867
        joint4 = 4.080136938686035
        joint5 = 5.042082786560059
        joint6 = 2.4556498527526855

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm1.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))


                 
        # 控制机械臂完成运动
        arm1.go()

        rospy.sleep(2)


        # 第一段 预抓取点
        joint1 = -6.257366005574362
        joint2 = -2.698485036889547
        joint3 = -2.2992353439331055
        joint4 = 4.870590674668112
        joint5 = 4.744915008544922
        joint6 = 2.425325870513916

        joint_positions = [joint1, joint2, joint3, joint4, joint5, joint6]
        result=arm1.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))


                 
        # 控制机械臂完成运动
        arm1.go()

        rospy.sleep(2)

        cartesian=1
        start_pose = arm1.get_current_pose(end_effector_link1).pose
        # 第二段走直线
        # 初始化路点列表
        waypoints1 = []
 
        # 如果为True,将初始位姿加入路点列表
        if cartesian:
            waypoints1.append(start_pose)
            
        # 设置路点数据，并加入路点列表，所有的点都加入
        wpose1 = deepcopy(start_pose)#拷贝对象
        wpose1.position.z -= 0.12
 
        if cartesian:  #如果设置为True，那么走直线
            waypoints1.append(deepcopy(wpose1))
        else:          #否则就走曲线
            arm1.set_pose_target(wpose1)  #自由曲线
            arm1.go()
            rospy.sleep(1)

        #规划过程
 
        if cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm1.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
        #规划路径 ，fraction返回1代表规划成功
            (plan, fraction) = arm1.compute_cartesian_path (
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
            arm1.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(2)

        left_dh3_open_publisher.left_gripper_open_publisher()
        rospy.sleep(2)

        # 0-5分别对应xyz rpy，平移单位为米，旋转单位为弧度
        # arm1.shift_pose_target(1, -0.05, end_effector_link1)
        # arm1.go()
        # rospy.sleep(1)


        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass