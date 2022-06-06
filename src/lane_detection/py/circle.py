#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class circle_drawer():
	def __init__(self):
		rospy.init_node("circle_drawer")
		self.pub = rospy.Publisher("circle_pc", PointCloud, queue_size=1)
		rospy.Timer(rospy.Duration(1.0), self.timer_CB)
		rospy.spin()

	def timer_CB(self, event):
		tmp_data = PointCloud()
		tmp_data.header.stamp = rospy.Time.now()
		tmp_data.header.frame_id = "velodyne"

		num_points = 200
		radius = 5

		for i in range(num_points):
			angle = i * 2 * math.pi / num_points
			tmp = Point32()
			tmp.x = radius * math.cos(angle)
			tmp.y = radius * math.sin(angle)
			tmp.z = 0
			tmp_data.points.append(tmp)

		self.pub.publish(tmp_data)
		rospy.loginfo("publish points")

if __name__ == '__main__':
	circle_drawer()


