#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
sys.path.append("/home/ck/ur_ws/src/dh_gripper_ros-master")
from dh_gripper_msgs.msg import GripperCtrl


def gripper_open_publisher():

    rospy.init_node('dh3', anonymous=Ture)
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)


    rate = rospy.Rate(10)
    count = 0 


    while not rospy.is_shutdown() or count != 0:
        gripper_state = GripperCtrl()
        
        gripper_state.position = 100
        gripper_state.force = 100
        gripper_state.speed = 100


        pub.publish(gripper_state)


        rate.sleep()
        count +=1


if __name__=='__main__':
    try:
        gripper_open_publisher()
    except rospy.ROSInterruptException:
        pass
