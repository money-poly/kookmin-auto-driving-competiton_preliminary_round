#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from lane_detection.cfg import image_processing_tmpConfig

def callback(config, level):
    print(type(config))
    print(config)
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
    #       {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(image_processing_tmpConfig, callback)
    rospy.spin()