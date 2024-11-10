#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy
import numpy as np
from dh3_open_publisher_test import gripper_open_publisher
from dh3_close_publisher_test import gripper_close_publisher
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def initialize_arm():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('multi_stage_trajectory_demo', anonymous=True)

    arm = MoveGroupCommander('left_arm')
    arm.allow_replanning(True)
    arm.set_pose_reference_frame('world')
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    return arm

def detect_aruco_marker(target_id=None):
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if not ret:
        rospy.loginfo("无法读取摄像头图像")
        return None, None

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if target_id is None or marker_id == target_id:
                # 示例的简单位置估计，可替换为实际测量或估计
                tvecs = [0.1 + 0.05 * i, 0.1, 0.1]
                return tvecs, None
    return None, None

def estimate_pose_with_aruco_ros(target_id=None):
    bridge = CvBridge()
    tvecs, rvecs = None, None

    def image_callback(msg):
        nonlocal tvecs, rvecs
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if target_id is None or marker_id == target_id:
                    camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
                    dist_coeffs = np.zeros((4, 1))
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)
                    return

    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.sleep(1)
    return tvecs, rvecs

def compute_error_vector(current_pose, target_tvec):
    error_x = target_tvec[0] - current_pose.position.x
    error_y = target_tvec[1] - current_pose.position.y
    error_z = target_tvec[2] - current_pose.position.z
    error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)
    return (error_x, error_y, error_z), error_magnitude

def move_to_pose(arm, target_pose):
    arm.set_pose_target(target_pose)
    success = arm.go()
    return success

def move_to_corrected_pose(arm, error_vector):
    current_pose = arm.get_current_pose().pose
    corrected_pose = deepcopy(current_pose)
    correction_factor = 0.1
    corrected_pose.position.x += correction_factor * error_vector[0]
    corrected_pose.position.y += correction_factor * error_vector[1]
    corrected_pose.position.z += correction_factor * error_vector[2]
    return move_to_pose(arm, corrected_pose)

def perform_gripper_action(open_gripper=True):
    if open_gripper:
        gripper_open_publisher()
    else:
        gripper_close_publisher()

def execute_segment(arm, target_id=None, predefined_target=None, use_open_loop=False):
    if predefined_target:
        rospy.loginfo("移动到预定义目标位置")
        move_to_pose(arm, predefined_target)
        return

    rospy.loginfo("定位 ArUco 码 ID: {}".format(target_id))
    if use_open_loop:
        tvecs, _ = detect_aruco_marker(target_id)
    else:
        tvecs, _ = estimate_pose_with_aruco_ros(target_id)

    if tvecs is None:
        rospy.loginfo("未检测到目标")
        return

    current_pose = arm.get_current_pose().pose
    error_vector, error_magnitude = compute_error_vector(current_pose, tvecs)

    error_threshold = 0.02
    while error_magnitude > error_threshold:
        success = move_to_corrected_pose(arm, error_vector)
        if not success:
            rospy.loginfo("机械臂运动失败")
            break
        current_pose = arm.get_current_pose().pose
        error_vector, error_magnitude = compute_error_vector(current_pose, tvecs)

def main():
    arm = initialize_arm()

    try:
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        execute_segment(arm, target_id=1)
        perform_gripper_action(open_gripper=False)

        predefined_pose = Pose()
        predefined_pose.position.x = 0.2
        predefined_pose.position.y = 0.1
        predefined_pose.position.z = 0.3
        execute_segment(arm, predefined_target=predefined_pose, use_open_loop=True)

        execute_segment(arm, target_id=2)
        perform_gripper_action(open_gripper=True)

    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    main()
