import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import rubber as con

########코드 구조#########
# 우선 라이다값 0도를 기준으로 하여 차량의 변화를 관찰한다.
# 왼쪽 차선일때는 라이다 값을 0~180도 / 오른쪽 차선일 때는 라이다 값을 180~360도 사이의 값을 본다.
# 라이다 값을 바탕으로 상대차량의 상대 속도 및 거리를 측정한다.
# 계산했을 때 차량 대략 6m/s로 이동중 > 내 차량 정지시
# 라이다 인덱스는 0~360까지 차량 전방 기준 반시계방향으로 360개 나눈거임

#########사용 변수 선언##########
# bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지
# right_state = True
# left_state = 0
# front_car_vel = 0
# front_car_dis = 0
# side_car_flag = 0
# ranges = [100.0] * 180  # [수정] 최대 안전 확보 (기본값 100으로 초기화, 초기 None 문제 방지)
# ranges_yet = [0.0] * 180  # [수정] ranges와 같은 길이로 초기화
# last_time = 0
# start_overtake_flag = 0
# speed = 0
###############################

drive_fn = None


def cal_car_vel(con):
    #print("cal_vel")

    N = len(con.ranges)

    # 전방 인덱스 설정
    # if con.right_state == True:
    #     front_indices = range(85, 90)
    # elif con.left_state == True:
    #     front_indices = range(0, 5)
    # else:
    front_indices = range(87, 93)

    # 초기화
    if any(con.ranges_yet[i] == 0 for i in front_indices):
        for i in front_indices:
            con.ranges_yet[i] = con.ranges[i]
        return

    #print(con.ranges_yet[87:93])
    # 속도 계산 
    delta = sum(con.ranges[i] - con.ranges_yet[i] for i in front_indices) / len(front_indices)

    if(delta == 0):
        con.count += 1
    else:
        con.front_car_vel = round(delta*20 / con.count*0.15, 0)
        #print(delta)

    # 거리 계산
    con.front_car_dis = round(sum(con.ranges[i] for i in front_indices) / len(front_indices), 0)

    # 이전 거리 갱신
    for i in front_indices:
        con.ranges_yet[i] = con.ranges[i]

    # 결과 출력
    #print(f"front_car_vel: {con.front_car_vel}, front_car_dis: {con.front_car_dis}")

    # 측면 차량 확인
    if ((con.right_state == True) and (con.side_car_flag == False)):
        con.side_car_flag = 1 if all(x > 50 for x in con.ranges[0:50]) else 0 # 옆에 차가 없다면 플래그 ON.
    elif ((con.left_state == True) and (con.side_car_flag == False)):
        con.side_car_flag = 1 if all(x > 50 for x in con.ranges[130:180]) else 0
    else:
        con.side_car_flag = 0

    # 차량 속도 가감산
    if con.front_car_dis > 10:
        con.speed += 3
    elif con.front_car_dis <= 10:
        if con.front_car_vel < 0:
            con.speed -= 3
        elif con.front_car_vel >= 0:
            con.speed += 3

    # 추월 조건. 앞차량의 거리가 일정거리 이상이고, 옆 차량이 인식안된다면,,,

    if (con.front_car_dis >= 5) and (con.side_car_flag == 1): # 여기서 플레그로 main문에 들어가게 해야한다
        con.start_avoid_flag = True #회피 시작 !!!
        car_avoid(con)
    
    return con.speed * 0.6

def car_avoid(con):  # 차량회피 플레그 제어부부

    print("avoid")
    rate = rospy.Rate(100)

    if con.start_avoid_flag == True: #회피 플래그 켜지면

        if con.right_state == True: #차량이 오른쪽에 있다면 

            if any(x < 30 for x in con.ranges[140:180]): #오른쪽 끝에 걸치면 다시 차선 따라가기.
                con.go_back_flag = True
                print("R: back to lane")            

        elif con.left_state == True:

            if any(x < 30 for x in con.ranges[0:40]):
                con.go_back_flag = True
                print("L: back to lane")


