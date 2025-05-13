#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy, time
from sensor_msgs.msg import LaserScan

ranges = None

def lidar_callback(data):
    global ranges    
    ranges = data.ranges[0:360]

rospy.init_node('Lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

rospy.wait_for_message("/scan", LaserScan) #딱 한번만 받음.
print("Lidar Ready ----------")

while not rospy.is_shutdown():
        
    step = (len(ranges) // 36) * 2 #360 / 36 * 2 = 20
    print("Distance:", [round(d,1) for d in ranges[::step]]) #step 간격일때 소수점 1자리까지 반올림하여 출력
                                                             #ranges[::step] : 전체 중 step 간격으로 요소를 건너뛰며 가져옴. 
    time.sleep(0.5)
    
