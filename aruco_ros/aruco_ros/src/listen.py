#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import PoseStamped


def callback(data):
    print data
def listener():
    rospy.init_node('topic_subscriber')
    sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, callback)
    print sub

    print type(sub)
    rospy.spin()

if __name__ == '__main__':
    listener()
