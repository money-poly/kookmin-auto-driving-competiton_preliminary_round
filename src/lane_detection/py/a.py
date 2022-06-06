#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from b import*

def rotate_turtle():
	rospy.init_node('rotate_turtle',anonymous = False)
	vel_pub = rospy.Publisher('rotate_turtle',Twist,queue_size =10)
	vel_ref = Twist()
	while not rospy.is_shutdown():
		if pub.publish(flag) == 1:
			vel_ref.angular.z = 2
			vel_pub.publish(vel_ref)

if __name__ =="__main__":
    rotate_turtle()
