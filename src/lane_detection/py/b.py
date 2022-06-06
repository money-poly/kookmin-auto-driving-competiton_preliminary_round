#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

rospy.init_node('set_flag',anonymous = False)
pub = rospy.Publisher("set_flag",Int32)
rate = rospy.Rate(0.20) # 0.20hz
flag = 1
while not rospy.is_shutdown():
    pub.publish(flag)
    rate.sleep()

