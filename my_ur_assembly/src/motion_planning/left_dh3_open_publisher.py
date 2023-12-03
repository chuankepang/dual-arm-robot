#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import sys
sys.path.append("/home/ck/ur_ws/src/dh_gripper_ros-master")
from dh_gripper_msgs.msg import GripperCtrl
import time


def left_gripper_open_publisher():

    #rospy.init_node('dh3_open', anonymous=False)

    try:
    	rospy.init_node('dh3',anonymous=True)
    except rospy.exceptions.ROSException as e:
    	print("Node has already been initialized, do nothing")

    pub = rospy.Publisher('/left_gripper/dh_gripper_driver/left/gripper/ctrl', GripperCtrl, queue_size=10)


    rate = rospy.Rate(10)
    count = 0 


    while(1):
        gripper_state = GripperCtrl()
        
        gripper_state.position = 60
        gripper_state.force = 100
        gripper_state.speed = 100

        time.sleep(0.5)
        pub.publish(gripper_state)

        count +=1
        if(count==1):
            break
    #rospy.signal_shutdown("closed")


if __name__=='__main__':
    try:
        left_gripper_open_publisher()
    except rospy.ROSInterruptException:
        pass
