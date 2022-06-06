#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray,Int32
from xycar_msgs.msg import xycar_motor
from math import*
class RobotController(object):
    def __init__(self):
        rospy.Subscriber("steering_angle", Float32MultiArray, self.steering_angle_callback)
        self.drive_pub = rospy.Publisher("/xycar_motor" ,xycar_motor,queue_size=1 )
        rospy.Subscriber('pole_curve',Int32,self.SteepCurve)
        self.data = None
    def steering_angle_callback(self, data):
        self.z_data = data.data # list
        # print(self.z_data)
        self.right_distance = self.z_data[0]
        self.left_distance = self.z_data[1]
        self.avg_val = self.z_data[2]
        # 차선의 기울기
        self.dist_steer = self.z_data[3]
        # 좌우 차선
        # print("right_distance : {}px".format(self.right_distance))
        # print("left_distance : {}px ".format(self.left_distance))
        # print("avg_val : {} ".format(round(self.avg_val,4)))
        # print("dist_steer : {}px".format(self.dist_steer))
        if self.data == 1 :

            self.Pole_Curve()
        
        elif abs(self.avg_val * (0.6)) > 5:

            self.curve()

        # elif abs(self.avg_val * (0.6)) < 1:
        #     self.straight()

        else:

            print("std mode")
            motor_data = xycar_motor()
            motor_data.speed = 10
            motor_data.angle= round( (self.dist_steer * -0.001 + self.avg_val*(0.024) )* 180 / pi ,4)
            print("self.dist_steer",round(self.dist_steer * -0.001* 180 / pi))
            print("avg_val",round(self.avg_val * (0.020) * 180 / pi))
            self.drive_pub.publish(motor_data)

        # motor_data.angle = self.avg_val*(-0.05)

        # self.steering_pub.publish(avg_val*(-0.5))
        # self.steering_pub.publish(dist_steer * 0.06)
    
        # rospy.loginfo("{} data published".format(self.z_data))
    def curve(self ):
        print("curve mode ")
        motor_data = xycar_motor()
        motor_data.speed = 10
        motor_data.angle= round((self.dist_steer * -0.001 + self.avg_val*(0.02) )* 180 / pi ,4)
        # print("angle_speed(degree/s) = {}".format(motor_data.angle))
        # print("current_x_speed : {} ".format(motor_data.speed)
        print("self.dist_steer",self.dist_steer * -0.0015 * 57)
        print("avg_val",self.avg_val*(0.02)* 180 / pi)
        self.drive_pub.publish(motor_data)

    def Pole_Curve(self):
        print("#################Steep curve mode############ ")
        motor_data = xycar_motor()
        motor_data.speed = 10
        motor_data.angle= round((self.dist_steer * -0.001 + self.avg_val*(0.026) )* 180 / pi ,4)
        # print("angle_speed(degree/s) = {}".format(motor_data.angle))
        # print("current_x_speed : {} ".format(motor_data.speed)
        print("self.dist_steer",self.dist_steer * -0.0001 * 57)
        print("avg_val",self.avg_val*(0.025)* 180 / pi)
        self.drive_pub.publish(motor_data)

    def SteepCurve(self,data):

        self.data = data.data
 


    def straight(self):
        print("straight mode ")
        motor_data = xycar_motor()
        motor_data.speed = 5
        motor_data.angle= round((self.dist_steer * 0.0005 + self.avg_val*(0.01)) * 180 / pi ,4)
        print("angle_speed(degree/s) = {}".format(motor_data.angle))
        print("current_x_speed : {} ".format(motor_data.speed))
        # self.drive_pub.publish(motor_data)

def run():
    rospy.init_node("robot_controller")
    robot_controller= RobotController()
    rospy.spin()
    
if __name__ == "__main__":
    run()
    
# /wecar/steering_angle Topic Subscribe (std_msgs/Float32) 
# /cmd_vel Topic Publish (geometry_msgs/xycar_motor)

## /wecar/steering_angle -->  xycar_motor.angle
## xycar_motor.angle = /wecar/steering_angle