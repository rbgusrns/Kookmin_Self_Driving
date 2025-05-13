#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2, rospy, numpy as np 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #ROS 이미지 메시지 <-> OpenCV 이미지 변환.
bridge = CvBridge() #변환 매개체
cv_image = np.empty(shape=[0])
'''
np.empty(...) : 초기값 없이 메모리만 잡고 배열을 만듦.
shape = [0] : 만들 배열의 형태를 지정. [0]은 0개의 요소를 가진 1차원 배열.
실제로는 np.empty((0,)) 이것과 동일함.
->크기도 없는 빈 배열. cv_image = []
'''

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8") #이미지 데이터 토픽을 받아와 cv로 변환. 색상 형식은 bgr 8BIT 형식

rospy.init_node('cam_test', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
               #최상위 토픽  하위 토픽
rospy.wait_for_message("/usb_cam/image_raw/", Image)
print("Camera Ready --------------")

while not rospy.is_shutdown():
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) #컬러 이미지를 흑백(그레이스케일) 이미지로 변환. 흑백은 계산이 빠름..
    #cvtColor(변환할 이미지, 어떤 색공간으로 바꿀지 지정하는 코드) : 이미지의 색상 공간을 변환하는 함수
    cv2.imshow("original", cv_image)
    cv2.imshow("gray", gray)
    cv2.waitKey(1) #1ms동안 멈춘 후 반복. 꼭 써야함.