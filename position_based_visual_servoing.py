#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from copy import deepcopy

def initialize_arm():
    """
    初始化机械臂控制环境和参数。
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('aruco_visual_servo', anonymous=True)

    arm = MoveGroupCommander('left_arm')
    arm.allow_replanning(True)
    arm.set_pose_reference_frame('world')
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    return arm

def detect_aruco_pose(frame, target_id=None):
    """
    检测图像中的 ArUco 码，并估计其位姿。
    参数:
        - frame: 图像帧
        - target_id: 目标 ArUco 码 ID (可选)
    返回:
        - tvecs: 平移向量
        - rvecs: 旋转向量
    """
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if target_id is None or marker_id == target_id:
                camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  # 示例相机矩阵
                dist_coeffs = np.zeros((4, 1))  # 示例值
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)
                return tvecs[0][0], rvecs[0][0]
    return None, None

def get_frame_from_camera():
    """
    从摄像头捕获图像帧。
    """
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    if ret:
        return frame
    else:
        rospy.loginfo("无法读取摄像头图像")
        return None

def compute_visual_servo_error(current_pose, target_tvec):
    """
    计算机械臂当前位置与目标位姿的误差。
    返回：
        - error_vector: 三维误差向量 (x, y, z)
        - error_magnitude: 误差向量的模
    """
    error_x = target_tvec[0] - current_pose.position.x
    error_y = target_tvec[1] - current_pose.position.y
    error_z = target_tvec[2] - current_pose.position.z
    error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)
    return (error_x, error_y, error_z), error_magnitude

def visual_servo_control(arm, error_vector, gain=0.1):
    """
    执行基于视觉伺服的机械臂控制。
    参数:
        - arm: MoveIt 的机械臂控制对象
        - error_vector: 当前位置与目标之间的误差向量
        - gain: 控制增益，用于调整步长
    返回:
        - success: 控制是否成功
    """
    current_pose = arm.get_current_pose().pose
    corrected_pose = deepcopy(current_pose)

    corrected_pose.position.x += gain * error_vector[0]
    corrected_pose.position.y += gain * error_vector[1]
    corrected_pose.position.z += gain * error_vector[2]

    arm.set_pose_target(corrected_pose)
    success = arm.go()
    return success

def aruco_visual_servo_tracking(arm, target_id=None):
    """
    实现基于 ArUco 码的视觉伺服控制。
    """
    while not rospy.is_shutdown():
        frame = get_frame_from_camera()
        if frame is None:
            rospy.loginfo("无法获取摄像头图像，停止跟踪")
            break

        tvecs, rvecs = detect_aruco_pose(frame, target_id)
        if tvecs is None:
            rospy.loginfo("未检测到目标 ArUco 码")
            continue

        current_pose = arm.get_current_pose().pose
        error_vector, error_magnitude = compute_visual_servo_error(current_pose, tvecs)

        error_threshold = 0.01
        if error_magnitude > error_threshold:
            success = visual_servo_control(arm, error_vector)
            if not success:
                rospy.loginfo("机械臂运动失败")
                break
        else:
            rospy.loginfo("已接近目标位置")

def main():
    arm = initialize_arm()

    try:
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        rospy.loginfo("开始视觉伺服控制")
        aruco_visual_servo_tracking(arm, target_id=1)

    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    main()

