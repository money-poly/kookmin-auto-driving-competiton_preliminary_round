#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
from dynamic_reconfigure.server import Server
from lane_detection.cfg import lidar_e_stopConfig

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=False)
        srv = Server(lidar_e_stopConfig, self.configure_cb)
        self.sub_ls = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.warn_pub = rospy.Publisher("/lidar_warning", String, queue_size=3)

    def configure_cb(self, config, level):
        self.E_STOP_MIN_ANGLE_DEG = config.e_stop_min_angle_deg
        self.E_STOP_MAX_ANGLE_DEG = config.e_stop_max_angle_deg
        self.E_STOP_DISTANCE_METER = config.e_stop_distance_meter
        self.E_STOP_COUNT = config.e_stop_count
        return config

    def callback(self, data):
        cnt = 0
        angle_rad = [data.angle_min + i * data.angle_increment for i, _ in enumerate(data.ranges)]
        angle_deg = [180 / math.pi * angle for angle in angle_rad]
        for i, angle in enumerate(angle_deg):
            if self.E_STOP_MIN_ANGLE_DEG <= angle <= self.E_STOP_MAX_ANGLE_DEG and data.ranges[i] < self.E_STOP_DISTANCE_METER:
                cnt += 1
        if cnt >= self.E_STOP_COUNT:
            self.warn_pub.publish("Warning")
            rospy.loginfo("Object Detected!! Warning!!")
        else:
            self.warn_pub.publish("Safe")
            rospy.loginfo("Safe!!")

if __name__ == "__main__":
    od = ObjectDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("program down")
