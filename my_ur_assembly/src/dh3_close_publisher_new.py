#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from dh_gripper_msgs.msg import GripperCtrl


def gripper_close_publisher():

    rospy.init_node('dh3', anonymous=Ture)
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)


    #rate = rospy.Rate(10) 


    #while not rospy.is_shutdown():
    gripper_state = GripperCtrl()
        
    gripper_state.position = 0;
    gripper_state.force = 100;
    gripper_state.speed = 100;


    pub.publish(gripper_state)
    rospy.spin()


    #rate.sleep()


if __name__=='__main__':
    try:
        gripper_close_publisher()
    except rospy.ROSInterruptException:
        pass
