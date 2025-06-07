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
# 라이다 인덱스는 0~180까지 차량 전방 기준 왼쪽 90도 부터 오른쪽 90도 까지임
# 차선 변경 시작은 전방 라이다 값으로 200 이하가 되었을 때 시작
# 좌우측 라이다 값에 차량이 인식되지 않았을 때 차량 회피 시작
# 회피도중 차량의 좌우측 라이다 90도 부근에 추월하고자 하는 차량 걸리면 다시 라인 복귀

drive_fn = None

'''
"cal_car_vel"
>> 차량의 상대속도 및 상대거리 측정 후 차간 거리 유지 및 회피 주행 스타트 플레그 설정 함수
>> 라이다 센서의 피드백 형식의 코드를 이용하여 상대 거리 측정
>> 라이다 callback 함수 주기를 측정하여 상대 속도 측정
>> 상대 거리 9 이상, 좌우측 차량 존재하지 않을 때 차량 회피 시작
'''
def cal_car_vel(con):
    #print("cal_vel")

    N = len(con.ranges)

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
    
    # 차량 속도 제한 !
    if con.speed >= 80: 
        con.speed = 80
    elif con.speed <= -80:
        con.speed = -80

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

            if any(x < 60 for x in con.ranges[140:180]): #오른쪽 끝에 걸치면 다시 차선 따라가기.
                con.go_back_flag = True
                print("R: back to lane")            

        elif con.left_state == True:

            if any(x < 60 for x in con.ranges[0:40]):
                con.go_back_flag = True
                print("L: back to lane")


