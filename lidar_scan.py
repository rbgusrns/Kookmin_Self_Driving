#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan

lidar_points = None

def lidar_callback(data):
    global lidar_points
    lidar_points = data.ranges
    # 0~359개의 라이다 거리 데이터 배열
rospy.init_node('Lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

while not rospy.is_shutdown():
    if lidar_points == None:
        continue
    
    rtn = ""
    for i in range(12):
        rtn += str(format(lidar_points[i*30],'.2f')) + ", "
        # i*30 : 총 12개 지점의 라이다 거리값을 선택 0,30,60...
        # format(...,'.2f') : 소수점 둘째 자리까지 문자열로 반환
        # str(...) + ", " : 문자열 형태로 만들고, 콤마와 공백을 붙임
        # rtn += : 결과 문자열에 이어 붙임
        
    print(rtn[:-2]) #맨 마지막 콤마와 공백 삭제
    time.sleep(0.5)
    
