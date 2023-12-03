#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf import TransformListener
 
 
class MoveItIkDemo:
    def __init__(self):

        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                


	listener = TransformListener()
        #需要修改
	(trans, rot) = listener.lookupTransform("world", "aruco_marker_frame", rospy.Time(0))

 
	rospy.loginfo(trans)
 
if __name__ == "__main__":
    MoveItIkDemo()

