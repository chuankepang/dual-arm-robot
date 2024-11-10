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
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('aruco_tracking_demo', anonymous=True)

    arm = MoveGroupCommander('left_arm')
    arm.allow_replanning(True)
    arm.set_pose_reference_frame('world')
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    return arm

def detect_aruco(frame, target_id=None):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if target_id is None or marker_id == target_id:
                camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  # 示例值，替换为实际相机参数
                dist_coeffs = np.zeros((4, 1))  # 示例值，替换为实际值
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)
                return tvecs[0][0], rvecs[0][0]
    return None, None

def get_frame_from_camera():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    if ret:
        return frame
    else:
        rospy.loginfo("无法读取摄像头图像")
        return None

def compute_error_vector(current_pose, target_tvec):
    error_x = target_tvec[0] - current_pose.position.x
    error_y = target_tvec[1] - current_pose.position.y
    error_z = target_tvec[2] - current_pose.position.z
    error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)
    return (error_x, error_y, error_z), error_magnitude

def move_to_corrected_pose(arm, error_vector):
    current_pose = arm.get_current_pose().pose
    corrected_pose = deepcopy(current_pose)
    correction_factor = 0.1
    corrected_pose.position.x += correction_factor * error_vector[0]
    corrected_pose.position.y += correction_factor * error_vector[1]
    corrected_pose.position.z += correction_factor * error_vector[2]
    arm.set_pose_target(corrected_pose)
    success = arm.go()
    return success

def track_aruco_marker(arm, target_id=None):
    while not rospy.is_shutdown():
        frame = get_frame_from_camera()
        if frame is None:
            rospy.loginfo("无法获取摄像头图像，停止跟踪")
            break

        tvecs, rvecs = detect_aruco(frame, target_id)
        if tvecs is None:
            rospy.loginfo("未检测到目标 ArUco 码")
            continue

        current_pose = arm.get_current_pose().pose
        error_vector, error_magnitude = compute_error_vector(current_pose, tvecs)

        error_threshold = 0.02
        if error_magnitude > error_threshold:
            success = move_to_corrected_pose(arm, error_vector)
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

        rospy.loginfo("开始跟踪 ArUco 码")
        track_aruco_marker(arm, target_id=1)

    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    main()
