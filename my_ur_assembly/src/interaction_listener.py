#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
import cv2
from handutil import HandDetector
import numpy as np

# shujuchuli
def xyCallback(data):
    rospy.loginfo("\n position \n x : %f \n y : %f \n z : %f \n orientation \n x : %f \n y : %f \n z : %f \n w : %f", data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

def xy_subscriber():
	# ROS节点初始化
    rospy.init_node('xypose_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/laneline_info的topic，注册回调函数lanelineInfoCallback
    rospy.Subscriber("chatter", Pose, xyCallback)


	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    xy_subscriber()


    
