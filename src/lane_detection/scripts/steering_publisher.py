#! /usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Int32
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from random import * 

class LaneDetection:
    def __init__(self):
        rospy.init_node("steering_node")
        self.cvbridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw", Image, self.Image_CB)
        self.steering_pub = rospy.Publisher("steering_angle", Float32MultiArray, queue_size=5)
        self.cmd_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.pole_curve = rospy.Publisher("pole_curve",Int32,queue_size = 1)
        self.src = np.array([[100,350],[540,350],[640,400],[0,400]],dtype=np.float32)
        self.dst = np.array([[0,0],[640,0],[640,480],[0,480]],dtype=np.float32)
        self.ORIGINAL_PTS = np.float32([[100,350],[540,350],[640,400],[0,400]])
        self.matrix = cv2.getPerspectiveTransform(self.src, self.dst)
        self.inv_matrix = cv2.getPerspectiveTransform(self.dst, self.src)
        self.REF_LINE_X = 320
        self.dist_lst = []

    def Image_CB(self, img):

        frame = self.cvbridge.imgmsg_to_cv2(img, "bgr8")


        bird_eye_image = self.bird_eye(frame)
        colored_image = self.color_detect(bird_eye_image)

        self.sliding_list_left = self.sliding_left(colored_image)
        self.sliding_list_right = self.sliding_right(colored_image)
        frame_steer = self.steering(frame, self.sliding_list_left, self.sliding_list_right)
        
        viz = True
        #visualization
        if viz:
            # 1.birdeye points
            arr1 = map(tuple, self.ORIGINAL_PTS)
            for i, pos in enumerate(arr1):
                cv2.circle(frame, pos, 5,(255,100 ,0), -1)
            # self.frame_pub.publish(self.cvbridge.cv2_to_imgmsg(frame,"bgr8"))
            # self.bev_pub.publish(self.cvbridge.cv2_to_imgmsg(bird_eye_image,"bgr8"))

            # 2.list points
            for i, pos in enumerate(self.sliding_list_left):
                x,y = pos
                cv2.circle(bird_eye_image, pos, 5,(255,0,0), -1)
                cv2.rectangle(bird_eye_image,(x-20,y-20),(x+20,y+20),(0,255,0),2)
            for i, pos in enumerate(self.sliding_list_right):
                x,y = pos
                cv2.circle(bird_eye_image, pos, 5,(0,255,0), -1)
                cv2.rectangle(bird_eye_image,(x-20,y-20),(x+20,y+20),(0,255,0),2)
            # self.result_pub.publish(self.cvbridge.cv2_to_imgmsg(cv2.warpPerspective(bird_eye_image, self.inv_matrix, (640, 480)),"bgr8"))        
            
            cv2.line(bird_eye_image, (self.REF_LINE_X, 0), (self.REF_LINE_X, 480), (0, 255, 0), 3, 1)

            cv2.circle(bird_eye_image,(self.REF_LINE_X + self.dist_steer,400), 10,(0,0,255), -1)
            cv2.circle(bird_eye_image,(self.REF_LINE_X,400), 10,(255,0,0), -1)
            cv2.line(bird_eye_image, (0, 400), (640, 400), (255, 255, 255),3,1)

            # for i in range(8):
            #     cv2.line(frame_image,(self.sliding_list_left[i]),(self.sliding_list_left[i+1]),(randint(0,255),randint(0,255),randint(0,255)),3,-1)
            #     cv2.line(frame_image,(self.sliding_list_right[i]),(self.sliding_list_right[i+1]),(randint(0,255),randint(0,255),randint(0,255)),3,-1)
            
            # cv2.line(bird_eye_image,(self.sliding_list_left[0]),(self.sliding_list_left[1]),(0,0,255),3,-1)
            # cv2.line(bird_eye_image,(self.sliding_list_left[1]),(self.sliding_list_left[2]),(0,0,255),3,-1)
            # cv2.line(bird_eye_image,(self.sliding_list_left[2]),(self.sliding_list_left[3]),(0,0,255),3,-1)
            # cv2.line(bird_eye_image,(self.sliding_list_left[3]),(self.sliding_list_left[4]),(0,0,255),3,-1)
            # cv2.line(bird_eye_image,(self.sliding_list_left[4]),(self.sliding_list_left[5]),(0,0,255),3,-1)
            # cv2.line(bird_eye_image,(self.sliding_list_left[5]),(self.sliding_list_left[6]),(0,0,255),3,-1)
            # cv2.line(bird_eye_image,(self.sliding_list_left[6]),(self.sliding_list_left[7]),(0,0,255),3,-1)

            # self.color_detect_pub.publish(self.cvbridge.cv2_to_imgmsg(colored_image))
            # self.bev_result_pub.publish(self.cvbridge.cv2_to_imgmsg(bird_eye_image,"bgr8"))        
            # result = cv2.warpPerspective(bird_eye_image, self.inv_matrix, (640, 480))

            cv2.imshow("frame", frame)

 
            cv2.imshow("bird_eye_and_colored_image", bird_eye_image)

            # cv2.setMouseCallback('bird_eye_and_colored_image',self.onMouse)
            # cv2.imshow("colored_image", colored_image)
            cv2.waitKey(1)
    def onMouse(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print( x , y )

    def steering(self, frame, sliding_list_left, sliding_list_right):
        x_left = []
        x_right = []
        self.REF_LINE_X = 320
        if len(sliding_list_left) == 0:
            left_distance = 0
            left_x = 0
        else:
            left_x = sliding_list_left[-1][0]
            left_distance = self.REF_LINE_X - sliding_list_left[-1][0] #should be positive
            # print("slidng_list_left[-1][0] : {}".format(sliding_list_left[-1][0]))
        if len(sliding_list_right) == 0:
            right_distance = 0
            right_x = 0
        else:
            right_x = sliding_list_right[-1][0]
            right_distance = self.REF_LINE_X - sliding_list_right[-1][0] #should be negative
        
        for i in range(0, len(sliding_list_left)): 
            x_left.append(sliding_list_left[i][0])
            # x 좌표만 추가하기
        # print("x_left",x_left)

        left_diff_arr = np.diff(x_left)
        # 각 배열의 차이 값 구하기
        divider = len(left_diff_arr)
        # 길이를 구하면 원래 배열보다 한 칸 작은 것이 나옴
        if divider == 0:
            divider = 1
        left_diff_sum = np.sum(left_diff_arr)
        left_avg = int(left_diff_sum/divider)
        
        # print("left_diff_arr{}".format(left_diff_arr))
        # print("left_diff_sum{}".format(left_diff_sum))
        # # print("left_avg{}".format(left_avg))

        #remove inappropriate cases 
        for i in range(0, len(left_diff_arr)): 
            if (left_diff_arr[i] > (left_avg +80)) or (left_diff_arr[i] < (left_avg - 80)): #the value 80 can be modified
                # 두 개의 점 차이가 평균 + 80 보다 크면 => 에러?
                self.sliding_list_left = []
                print("left!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
                left_diff_sum = 0

        #extract x coordinates from sliding_list_right
        for i in range(0, len(sliding_list_right)): 
            x_right.append(sliding_list_right[i][0])

        right_diff_arr = np.diff(x_right)
                # 각 배열의 차이 값 구하기
        divider = len(right_diff_arr)
        if divider == 0:
            divider = 1
        right_diff_sum = np.sum(right_diff_arr)
        right_avg = int(right_diff_sum/divider)

 
        emergency = Int32()
        no_emergency = Int32()

        emergency.data = 1
        no_emergency.data = 0

        # for x,y in sliding_list_left:
        #     max_left.append(x)
        # for x,y in sliding_list_right:
        #     min_right.append(x)

        if len(x_left) != 0 and len(x_right) != 0:
            if abs(max(x_left) - min(x_right)) < 120:
                if len(sliding_list_left) > len(sliding_list_right):
                    print("####################cross the center line ######################")
                    sliding_list_right = []
                    self.pole_curve.publish(emergency)

                else:
                    print("####################cross the center line l######################")
                    sliding_list_left = []
            else:
                self.pole_curve.publish(no_emergency)

        if len(sliding_list_left) == 0:
            print("no left list")
            print("one line avg",right_diff_sum * -1)
            avg_val = right_diff_sum * -1
        elif len(sliding_list_right) == 0:
            print("no right list")
            print("one line avg",left_diff_sum * -1 )
            avg_val = left_diff_sum * -1
        else:
            avg_val = float((left_diff_sum + right_diff_sum)/2) *-1

        #publish average values divided by 100
        left_dist_th = 320
        right_dist_th = -320


        # cmd_data = Twist()
        self.angle_inf = Float32MultiArray()
        lst= []
        self.angle_inf.data.append(right_distance)
        self.angle_inf.data.append(left_distance)


# 왼쪽 차이는 양수가 되어야 하고 오른쪽 차이느 음수가 되어야 한다.
        
        if left_distance == 0 and right_distance != 0:
            # dist_steer = right_distance - right_dist_th 
            dist_steer = -1 * self.dist_lst[-1]
        elif right_distance == 0 and left_distance != 0:
            # dist_steer = left_distance + left_dist_th
            dist_steer = self.dist_lst[-1] #나갔을때 반대 방향을 틀게 하려고 원래는 부호 반대 
        elif right_distance == 0 and left_distance == 0:
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            dist_steer = 0
            avg_val = 0
        else:
            dist_steer = right_distance + left_distance
            self.dist_lst.append(dist_steer)

#
        if avg_val > 100:
            avg_val = 100
        elif avg_val < -100:
            avg_val = -100

        avg_val /= 100
        avg_val *= 15

        self.angle_inf.data.append(avg_val)
        self.angle_inf.data.append(dist_steer)
        self.dist_steer = dist_steer
        self.steering_pub.publish(self.angle_inf)


        return frame #return frame is for visualization

    def sliding_left(self, img):
        left_list = []

        for j in range(0, img.shape[0] - 20, 50): # 100~460 40씩 증가
            j_list = []
            for i in range(0, 300, 5): # 0~300까지 5증가

                num_sum = np.sum(img[j - 20:j + 20, i - 19:i + 20]) #window size is 40*40
                # print(num_sum)
                if num_sum > 255 * 400: #pick (i,j) where its num_sum is over 100000
                    j_list.append(i)
                    # print("j , i  ,j_list",j,i,j_list)
            try:
                len_list = [] 
                #cluster if a gap between elements in the list is over 5
                result = np.split(j_list, np.where(np.diff(j_list) > 5)[0] + 1) 
                # print("result = ",result2
                # where 함수는 조건 만족하는인덱스 반환
                for k in range(0, len(result)):
                    len_list.append(len(result[k])) #append the lengths of each cluster
                largest_integer = max(len_list)
            
                for l in range(0, len(result)):
                    if len(result[l]) == largest_integer: 
                        avg = int(np.sum(result[l]) / len(result[l])) #average
                        # print(avg)
                        left_list.append((avg, j)) #avg points of left side

            except:
                continue
        return left_list

    
    def sliding_right(self, img):
        right_list = []

        for j in range(0, img.shape[0] - 20, 50 ): 
            j_list = []
            for i in range(int(img.shape[1]/2)+ 20 , 640, 5): #img.shape[1] - 20
                num_sum = np.sum(img[j - 19:j + 20, i - 20:i + 20]) #window size is 40*40
                if num_sum > 255 * 400: #pick (i,j) where its num_sum is over 100000
                    j_list.append(i)
            try:
                len_list = []
                #cluster if a gap between elements in the list is over 5
                result = np.split(j_list, np.where(np.diff(j_list) > 5)[0] + 1) 
                for k in range(0, len(result)):
                    len_list.append(len(result[k])) #append the lengths of each cluster
                largest_integer = max(len_list)
            
                for l in range(0, len(result)):
                    if len(result[l]) == largest_integer: 
                        avg = int(np.sum(result[l]) / len(result[l])) #average
                        right_list.append((avg, j)) #avg points of left side 
            except:
                continue
        return right_list 


    def color_detect(self, img):
        # print(self.YELLOW_LANE_LOW)
        # print(self.YELLOW_LANE_HIGH)
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        Lchannel = hls[:,:,1]
        mask = cv2.inRange(Lchannel,240,255)
        res = cv2.bitwise_and(img,img,mask=mask)

        return res

    def bird_eye(self, frame):
        result = cv2.warpPerspective(frame, self.matrix, (640, 480))
        return result


if __name__ == '__main__':
    a = LaneDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("program down")










#   def calculate_curvature(self, print_to_terminal=False):
#     """
#     Calculate the road curvature in meters.

#     :param: print_to_terminal Display data to console if True
#     :return: Radii of curvature
#     """
#     # Set the y-value where we want to calculate the road curvature.
#     # Select the maximum y-value, which is the bottom of the frame.
#     y_eval = np.max(self.ploty)    

#     # Fit polynomial curves to the real world environment
#     left_fit_cr = np.polyfit(self.lefty * self.YM_PER_PIX, self.leftx * (
#       self.XM_PER_PIX), 2)
#     right_fit_cr = np.polyfit(self.righty * self.YM_PER_PIX, self.rightx * (
#       self.XM_PER_PIX), 2)
			
#     # Calculate the radii of curvature
#     left_curvem = ((1 + (2*left_fit_cr[0]*y_eval*self.YM_PER_PIX + left_fit_cr[
#                     1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
#     right_curvem = ((1 + (2*right_fit_cr[
#                     0]*y_eval*self.YM_PER_PIX + right_fit_cr[
#                     1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
	
#     # Display on terminal window
#     if print_to_terminal == True:
#       print(left_curvem, 'm', right_curvem, 'm')
			
#     self.left_curvem = left_curvem
#     self.right_curvem = right_curvem

#     return left_curvem, right_curvem	