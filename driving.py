#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from xycar_msgs.msg import XycarMotor
from math import *
import signal
import sys
import os
import random
from sinhodeng import sinho_detect #신호등 감지 함수 받아옴
from rubber import CurveNavigator
import matplotlib.pyplot as plt
import avoid_wd_lidar as avoid # 차량 회피 받아옴

Rubber = CurveNavigator()

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수
lx,ly,rx,ry = [], [], [], []
# GAUNSSIAN_BLUR_FUNCTION CONST
KERNEL_SIZE = 5

#노란색 차선 검출.
LOW_YELLOW  = np.array([20, 100, 100])
HIGH_YELLOW = np.array([30, 255, 255])

# HSV_FILTERED CONST
LOW_WHITE = np.array([0, 0, 50])
HIGH_WHITE = np.array([0, 0, 255])

# CANNY_EDGE_DETECTION_FUNCTION CONST
LOW_THRESHOLD = 100
HIGH_THRESHOLD = 200

# BIRD_EYE_VIEW_FUNCTION CONST
SOURCE_POINTS = np.float32([[210, 300], [40, 438], [425, 300], [595, 453]])
#                              L_UP       L_DOWN      R_UP       R_DOWN      
DESTINATION_POINTS = np.float32([[160, 10], [160, 470], [480, 10], [480, 470]])
#                                 L_UP        L_DOWN       R_UP      R_DOWN

# HISTOGRAM_FUNCTION CONST	    # Sampling window 에서 사용하는 상수값을 저장하는 변수.
nwindows = 8			    # 샘플링할 사각형의 개수.
margin = 80			        # 샘플링한 사각형의 넓이.
minpix = 5			        # 검출된 선 배열 기준 값.
lane_bin_th = 80		    # cv2.threshold 함수의 thresh 지정 임계값. 						

#DRIVE				        # 주행에 사용되는 상태 확인 변수 및 상수값을 저장하는 변수.
degree = 0			        # 차선에서 샘플링된 상자들중 두 값을 각각 골라 평균값들의 도출된 각도를 저장하는 변수.
angle = 0
error = 0
START_BOX = 1			    # 샘플링하는 두 상자중 차량에서 가까운 상자를 결정하는 상수값을 저장하는 변수.
TARGET_BOX = 7			    # 샘플링하는 두 상자중 차량에서 먼 상자를 결정하는 상수값을 저장하는 변수.
CONFIG_ERROR = 8		    # 차량의 목표 진행각도와 비교해 직선과 곡선을 결정하는 상수값을 저장하는 변수.
CONFIG_DEGREE = 1.0         # 차량의 목표 진행각도와 비교해 직선과 곡선을 결정하는 상수값을 저장하는 변수.
MAX_DEGREE = 50			    # 목표 진행각도의 최고값을 결정하는 상수값을 저장하는 변수.
DX_GAIN = 0.27              # P_GAIN : 각 설정된 상자의 평균X 값의 차인 DX값에 어느 정도를 곱해주어 PD제어에 이용할지를 결정한는 상수값을 저장하는 변수.																	
DEGREE_GAIN = 0.203		    # D_GAIN : degree 값에 어느 정도를 곱해주어 PD제어에 이용할지를 결정한는 상수값을 저장하는 변수.

LANE_WIDTH = 320		    # BIRD_EYE_VIEW 를 통해 변환된 차선의 폭 값인 상수값을 저장하는 변수.
driving_flag = 0            # 차량의 주행 여부를 구분할 변수.
straight_flag = 1		    # 차선의 직선 여부를 구분할 변수.
curve_flag = 0			    # 차선의 곡선 여부를 구분할 변수.
right_flag = 0			    # 차선의 우회전 여부를 구분할 변수.
left_flag = 0 			    # 차선의 좌회전 여부를 구분할 변수.
uphill_flag = 0			    # 도로의 방지턱 여부를 구분할 변수.
STRAIGHT_VELO = 100		# 차선이 직선일 경우의 속력.
TURN_VELO = 55		    # 차선이 곡선일 경우의 속력. 

AVOID_VEL = TURN_VELO
AVOID_ANGLE = 40
TURN_FASTVEL = TURN_VELO + 30
straight_vel = STRAIGHT_VELO
turn_vel = TURN_VELO
#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#============================================= 	
# 추가된 함수 
# 차선 정보를 입력받은 뒤 PD 차선의 종류를 구분하고 PD 제어를 통해 조향각을 변경한다.
#=============================================
def histogram(lane):
    global rx
    global lx
    global ry
    global ly


    # lane정보에서 차선인식을 원할하게 하기 위해 histogram화.
    histogram = np.sum(lane[lane.shape[0] // 2:,:], axis=0)
    # Numpy 배열에서, .shape는 배열의 차원을 튜플(height, width)로 반환한다.. 
    # ex) (세로,가로) -> (480,640)이므로 lane.shape[0] = 이미지의 세로 길이. 그런데 이것의 절반부터 시작한다고 하니, 화면 절반 아래를 의미. 가로는 전부임.
    # 결국 lane[절반 하단, 가로 전체]인데, np.sum(..., axis=0)에서 axis가 0이라면 세로방향으로 더한다는 뜻.
    # 즉, histogram은 1차원 배열인데, x = 0 ~ 640 일때 각 열(세로)의 픽셀 값 총합임. 밝은걸 더하겠지..? 
    # 중심좌표 찾기
    midpoint = np.int(histogram.shape[0] / 2) # x의 중심 좌표. 320정도..
    # 왼쪽과 오른쪽 좌표 찾기
    leftx_current = np.argmax(histogram[:midpoint])  #왼쪽 중에서, 가장 밝기가 밝은 곳의 x좌표 인덱스 찾음.  
    
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint #절반부터 0인 인덱스이니, 원래 인덱스 찾기.
    
    #맨 아래(차랑 가장 가까운) 윈도우의 x좌표를 잡아줌. 이후로 반복문을 돌며 현재의 차선을 기준으로 다음 윈도우를 설정.
    #np.argmax() : 배열중에서 최댓값 인덱스 반환 값이 여러개라면 가장 빠른 인덱스 반환
    # 창의 높이 설정
    window_height = np.int(lane.shape[0] / nwindows)
    
    # 선이 있는 부분의 인덱스만 저장
    nz = lane.nonzero()
    # lane은 이진화 이미지. 2차원 배열로 이루어져 있다..
    # lane.nonzero() : 0이 아닌 픽셀, 즉 차선의 인덱스들을 반환함. 반환값은 튜플 2개이며, (y인덱스 배열, x인덱스 배열) ( 행 ,열 )
    # 인덱스 저장을 위한 공간 생성
    left_lane_inds = []
    right_lane_inds = []

    lx,ly,rx,ry = [], [], [], []
    l_count = 0
    r_count = 0 # 현재 왼쪽인지 오른쪽인지 구분하기 위한 변수.
    out_img = np.dstack((lane,lane,lane))*255
    # window를 생성, 차선 시각화
    for window in range(nwindows): #y값은 아래로 내려갈 수록 커진다. window가 0일때 가장 아래임. 세로:480, 윈도우 개수는 8이므로 크기는 60
        # window 좌표
        win_yl = lane.shape[0] - (window+1)*window_height # 420
        win_yh = lane.shape[0] - window*window_height # 480

        win_xll = leftx_current-margin 
        win_xlh = leftx_current+margin
        win_xrl = rightx_current-margin
        win_xrh = rightx_current+margin

        # window 그리기. 시각화
        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0),2)
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0),2)
        
        #각각의 window 안에 있는 부분만을 저장. 인덱스형태로 저장한다. nz[0]은 y인덱스, nz[1]은 x인덱스.
        good_left_inds = ((nz[0]>=win_yl)&(nz[0]<win_yh)&(nz[1]>=win_xll)&(nz[1]<win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0]>=win_yl)&(nz[0]<win_yh)&(nz[1]>=win_xrl)&(nz[1]<win_xrh)).nonzero()[0]
        #                 (~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~) (~~~~~~~)(~)
        #                       [True,False,False...]의 불리언 배열. 윈도우 안에 있는가?     의 인덱스   중   
        # 검출된 선 좌표를 배열의 끝에 추가.

        #if window == 3 or window == 2:
        #    print(good_right_inds)
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        #차선이 끊겨 있을 경우 good_left_inds,good_right_inds는 빈 배열임

        if len(good_left_inds)>minpix:
                leftx_current = np.int(np.mean(nz[1][good_left_inds])) #윈도우에서 처선이라고 생각되는 부분의 중앙 x 좌표
                l_count += 1
                
        if len(good_right_inds)>minpix:
                rightx_current = np.int(np.mean(nz[1][good_right_inds]))
                r_count += 1
                
        #만약 차선이 끊겨있을 경우 leftx_current,rightx_current는 기존의 값을 유지함. 
        #윈도우는 아래부터 위로 검사하기 때문에 차선이 없을 경우 필연적으로 새로운 윈도우가 빈 배열임.
        #따라서 차선이 존재할때 곡률을 정해놓아야 함.
        
        
        #만약 이전과 현재의 x가 같다면 차선을 인식 못한 것. 

        if (r_count>l_count): #만약 오른쪽의 차선이 왼쪽보다 많다면 지금 오른쪽 차선에 있다는 것.
            Rubber.right_state = True
            Rubber.left_state = False
            #print("R")

        elif (r_count<l_count):
            Rubber.left_state = True
            Rubber.right_state = False
            #print("L")

        else:
            pass
    
        lx.append(leftx_current)
        ly.append((win_yl+win_yh)/2)
        
        rx.append(rightx_current)
        ry.append((win_yl+win_yh)/2)

        #print(lx,rx)
        # 현재 차선이 끊겨있을 때, 이전의 x좌표를 그대로 물고감. 직선일 때는 상관없으나 곡선일 경우 경로 그리는데 문제 생김..
        # 곡률을 이어나가서 예상 좌표를 찍어야 함.

    # 배열을 1차원으로 합침
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    #print (lx,rx)
    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)
    # out_img 확인을 위해 그림 차선을 그림.
    out_img[nz[0][left_lane_inds],nz[1][left_lane_inds]]=[255,0,0]
    out_img[nz[0][right_lane_inds],nz[1][right_lane_inds]]=[0,0,255]
    #cv2.imshow("viewer", out_img)


# 곡선과 직선을 구분하는 함수.
def Turn_Configuration() :
    # 전역변수 선언
    global rx
    global lx
    global straight_flag
    global curve_flag
    global uphill_flag
    uphill_flag = 0
    # 오른쪽과 왼쪽 차선의 기울기 부호를 구하기 위해 각각 차선의 x 값들을 뺌
    #print (lx,rx)
    start_check = (rx[TARGET_BOX] - rx[START_BOX]) * (lx[TARGET_BOX] - lx[START_BOX])

    # 계산한 값이 음수이고 바깥으로 벌어지는 기울기인 경우에 동작.
    if start_check < 0 and (rx[TARGET_BOX] - rx[START_BOX]) > 0 :
        # 이 경우 직선으로 판단한 것. straight_flag = ON, curve_flag = OFF 으로 설정.
        #straight_flag = 1
        #curve_flag = 0
        pass
    # 계산한 값이 음수이고 안쪽으로 모아지는 기울기인 경우에 동작.        
    else :
        pass

    #print(straight_flag,curve_flag)

# 조향각을 결정하는 함수
def Steer_Configuration() :
    # 전역변수 선언
    global rx
    global lx

    # sliding_window 오류 처리
    # sliding_window 에서 사각형의 크기가 80. 차선으로 인식되는 좌표 평균이 50 미만인 경우 차선이 인식되지 않는걸로 처리.
    
    if (sum(lx) / len(lx)) < 30 :
        # 왼쪽 차선의 x 좌표 값들을 전부 0 으로 입력.
        lx = [0] * len(lx)

    # sliding_window 에서 사각형의 크기가 80. 차선으로 인식되는 좌표 평균이 640 초과인 경우 차선이 인식되지 않는걸로 처리.
    elif (sum(rx) / len(rx)) > 600 :
        # 오른쪽 차선의 x 좌표 값들을 전부 640 으로 입력.
        rx = [640] * len(rx) 

    else :
        pass
    
    # 차선의 없음을 인식하는 코드. 왼쪽 차선의 x 값들의 합이 0 또는 오른쪽 차>선의 x 값들의 합이 640인 경우에 empty_line_flag = ON.
    empty_line_flag = 1 if sum(lx) == 0  or sum(rx)/len(rx) == (WIDTH/2) else 0
    
	# 차선의 겹쳐짐을 인식하는 코드. 왼쪽 차선의 x 값>과 오른쪽 차선의 x 값이 같은 경우 fusion_line_flag = ON.
    fusion_line_flag = 1 if lx == rx else 0
    #print(empty_line_flag,fusion_line_flag)
    # 한쪽 또는 두쪽 선 출력 없음
    if empty_line_flag :
        
		# 차선이 한쪽만 인식되는 상황.
        # 오른쪽 차선이 인식되어 있고, 오른쪽 차선의 x 값이 화면 중앙의 값보다 크면 실행.
        if sum(rx) and (sum(rx) / len(rx)) > (WIDTH/2) :
             # 왼쪽 차선의 x 값을 오른쪽 차선 x 값에서 차선폭 길이만큼 빼줌. 
            lx[TARGET_BOX] = rx[TARGET_BOX] - LANE_WIDTH
            lx[START_BOX] = rx[START_BOX] - LANE_WIDTH

        # 왼쪽 차선이 인식되어 있고, 왼쪽 차선의 x 값이 화면 중앙의 값보다 작으면 실행.
        elif sum(lx) and (sum(lx) / len(lx)) < (WIDTH/2) : 
            # 오른쪽 차선의 x 값을 왼쪽 차선 x 값에서 차선폭 길이만큼 더해줌. 
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH 

        # 차선이 모두 인식되지 않는 상황.
        elif (sum(lx)/len(lx)) == 0  and ((sum(rx)/len(rx)) == (WIDTH/2)) : 
            # 왼쪽 차선과 오른쪽 차선의 값을 조정해 직선으로 주행하도록 함 
            lx = [160] * len(lx)
            rx = [480] * len(rx)
    else :
        pass
    
    
    # 두 선이 중복 출력될 경우  
    if fusion_line_flag : 
        
		# 기준이 되는 사각형 중 차량에서 먼쪽의 x 값이 화면의 중심의 값보다 작을 경우 실행.
        if rx[TARGET_BOX] < (350) : 
            # 왼쪽 차선의 x 값에 차선 폭만큼 더해 오른쪽 차선의 x 값에 대입. 
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH

        # 기준이 되는 사각형 중 차량에서 먼쪽의 x 값이 화면의 중심의 값보다 클 경우 실행.
        elif lx[TARGET_BOX] > (290) : 
            # 오른쪽 차선의 x 값에 차선 폭만큼 더해 왼쪽 차선의 x 값에 대입. 
            lx[TARGET_BOX] = rx[TARGET_BOX] - LANE_WIDTH
            lx[START_BOX] = rx[START_BOX] - LANE_WIDTH

    else :
        pass
    
    # 인식된 두 차선간 거리가 LANE_WIDTH * 0.8 보다 작은 경우
    if (rx[START_BOX] - lx[START_BOX]) < (LANE_WIDTH * 0.8) :
        
		# 사각형 먼쪽의 x의 평균값이 화면 넓이의 절반보다 작은 경우 실행.
        if ((rx[START_BOX] + lx[START_BOX]) / 2) < (WIDTH / 2) :
            # 왼쪽 차선의 x 값에 차선 폭만큼 더해 오른쪽 차선의 x 값에 대입.
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH

        # 사각형 먼쪽의 x의 평균갑이 화면 넓이의 절반보다 큰 경우 실행.
        elif ((rx[START_BOX] + lx[START_BOX]) / 2) > (WIDTH / 2) :
            # 왼쪽 차선의 x 값에 차선 폭만큼 더해 오른쪽 차선의 x 값에 대입.
            lx[TARGET_BOX] = rx[TARGET_BOX] - LANE_WIDTH
            lx[START_BOX] = rx[START_BOX] - LANE_WIDTH

    else : 
        pass
    #print(lx,rx)
    

# PD 제어를 위한 기본 설정 함수.
def PD_Setup() :
    # 전역변수 선언
    global degree
    global error
    global rx
    global lx
    global ry
    global ly	
    global DX
    global DY

    DX = ((rx[TARGET_BOX] + lx[TARGET_BOX]) / 2) - ((rx[START_BOX] + lx[START_BOX]) / 2)		      # DX : 차량에 가까운 x 좌표들의 평균의 합과 차량에 먼 x 좌표들의 평균의 합의 차.		
    DY = ((ry[START_BOX] + ly[START_BOX]) / 2) - ((ry[TARGET_BOX] + ly[TARGET_BOX]) / 2)		      # DY : 차량에 가까운 y 좌표들의 평균의 합과 차량에 먼 y 좌표들의 평균의 합의 차.
    degree =(math.atan2(DX , DY) * 180) / math.pi if DY else 0                                        # degree : 위에서 구한 DX, DY 를 통해 목표 지점을 향한 각도를 구함. 
    #print(degree,error)
    error = (WIDTH / 2) - (rx[TARGET_BOX] + lx[TARGET_BOX] + rx[START_BOX] + lx[START_BOX]) / 4       # error : 선택한 상자 4개의 x 좌표값의 평균과 화면 중앙 x 좌표값의 차.
    #print(error)
	# degree 의 값이 너무 커지는 경우를 막기 위>해 제한함. 계산된 degree 의 값의 크기가 지정해 둔 각도보다 크면 실행.
    if abs(degree) > MAX_DEGREE :
        # degree 의 부호에 따라 차량의 조향 방향이 다르므로 방향에 맞추어 최대 각도록 조향해 준다.
        degree = MAX_DEGREE if degree > 0 else -MAX_DEGREE

    else :
        pass

# 차선을 구분하기 위한 함수.
def Line_Configuration() :
    # 전역변수 선언
    global error
    global degree
    global straight_flag
    global curve_flag

    # degree 의 크기를 기준으로 직선과 곡선을 구분. degree 의 크기가 지정된 값보다 작으면 실행.
    if abs(degree) < CONFIG_DEGREE :
        # 이 경우 곡선이 아닌 직선으로 판단한 것. curve_flag = OFF, straight_flag = ON 으로 설정.
        curve_flag = 0
        straight_flag = 1

    # degree 의 크기가 지정된 값보다 큼을 뜻함.
    else :
        # 이 경우 곡선이 아닌 직선으로 판단한 것. curve_flag = ON, straight_flag = OFF 으로 설정.
        curve_flag = 1
        straight_flag = 0
    #print(error,straight_flag,curve_flag)
'''
    # error 의 크기를 기준으로 직선과 곡선을 구분. error 의 크기가 지정된 값보다 작으면 실행.
    if abs(error) < CONFIG_ERROR :
        # 이 경우 곡선이 아닌 직선으로 판단한 것. curve_flag = OFF, straight_flag = ON 으로 설정.
        curve_flag = 0
        straight_flag = 1

    # error 의 크기가 지정된 값보다 큼을 뜻함.
    else :
        # 이 경우 곡선이 아닌 직선으로 판단한 것. curve_flag = ON, straight_flag = OFF 으로 설정.
        curve_flag = 1
        straight_flag = 0
    print(error,straight_flag,curve_flag)
'''
    
# PD제어를 위한 함수.            
def PD_Control() :
    # 전역변수 선언 
    global angle
    global degree
    global error
    global straight_flag
    global curve_flag
    global right_flag
    global left_flag	

    # 곡선이 경우의 조향각을 계산함.
    #print(straight_flag,curve_flag)
    if curve_flag :

        right_flag = 1 if DX > 0 else 0                                  # DX 값이 양수인 경우 우회전임을 뜻하므로 right_flag = ON. 아닌 경우는 OFF.
        left_flag = 1 if DX < 0 else 0                                   # DX 값이 음수인 경우 좌회전임을 뜻하므로 left_flag = ON. 아닌 경우는 OFF.
        angle = abs(error * DX_GAIN + degree * DEGREE_GAIN )             # 조향각은 이미 구해둔 error, degree 를 이용하여 계산한다.
        angle = -angle if left_flag else angle                           # 절대값으로 계산하였고, degree 와 다르게 좌회전인 경우 음수 값을 가지므로 좌우 방향에 따른 설졍을 해줌.	
    
	# 직선으로 구분한 경우에도 목표 좌표로의 조향이 필요하므로 조향각을 계산함.
    else :
        #angle = (error * DX_GAIN*0.8 + degree * DEGREE_GAIN*0.8)*0.5

        right_flag = 1 if DX > 0 else 0                                  # DX 값이 양수인 경우 우회전임을 뜻하므로 right_flag = ON. 아닌 경우는 OFF.
        left_flag = 1 if DX < 0 else 0   
        angle = abs(error * DX_GAIN + degree * DEGREE_GAIN )
        angle = -angle if left_flag else angle
    #print(angle)
			

#=============================================
# 모터 토픽을 발행하는 함수  # 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = XycarMotor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    #print(angle,speed)
    motor.publish(motor_msg)

avoid.drive_fn = drive

def lane_follow(avoid_speed):
    #print("l")
    global motor, image
    global rx
    global lx
    global ry
    global ly
    global driving_flag
    global straight_vel
    global turn_vel
    global right_flag
    global left_flag

    raw_img = image.copy()  
    
    # HSV_FILTERED / 특정 색 검출 
    
    hsv_filtered = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV) # 밝기를 뽑아내기 위해 HSV로 변환.
    mask_yellow = cv2.inRange(hsv_filtered, LOW_YELLOW, HIGH_YELLOW) # 노란색 차선 검출
    mask_white = cv2.inRange(hsv_filtered, LOW_WHITE, HIGH_WHITE) # 하얀색 차선 검출
    combined_mask = cv2.bitwise_or(mask_yellow, mask_white) #필터 통합

    #cv2.imshow("VVViewer",hsv_filtered)
    hsv_result = cv2.bitwise_and(raw_img, raw_img, mask = combined_mask) # 하얀색과 노란색, 차선만을 뽑아낸 이미지
    #cv2.imshow("VVViewer",hsv_filtered)

    #pts = SOURCE_POINTS.reshape((-1, 1, 2)).astype(np.int32)
    #cv2.polylines(raw_img, [pts], isClosed=True, color=(0, 255, 255), thickness=2)
    #cv2.imshow("VViewer",raw_img)

    #dpts = DESTINATION_POINTS.reshape((-1, 1, 2)).astype(np.int32)
    #cv2.polylines(raw_img, [dpts], isClosed=True, color=(0, 255, 255), thickness=2)
    #cv2.imshow("VVviewer",hsv_result)


    # BIRD_EYE_VIEW_FUNCTION / 상공에서의 시점 처리
    transform_source = cv2.getPerspectiveTransform(SOURCE_POINTS, DESTINATION_POINTS)
    bird_eye_view = cv2.warpPerspective(hsv_result, transform_source, (640, 480))   

    # SLIDING_WINDOW / 선 인식
    _,L,_ = cv2.split(cv2.cvtColor(bird_eye_view, cv2.COLOR_BGR2HLS)) #HLS로 변환 후, L(밝기)만 뽑아냄

    #dpts = DESTINATION_POINTS.reshape((-1, 1, 2)).astype(np.int32)
    #cv2.polylines(L, [dpts], isClosed=True, color=(0, 255, 255), thickness=2)

    #cv2.imshow("VVVViewer", L)
    _,lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY) #일정 밝기 이상만 차선으로 검출
    #cv2.imshow("VVViewer", bird_eye_view)
    cv2.waitKey(1)       

    #=========================================
    # 핸들조향각 값인 angle값 정하기.
    # 차선의 위치 정보를 이용해서 angle값을 설정함.        
    #=========================================
    # 차선 정보를 불러옴 	
    histogram(lane)
    Turn_Configuration()

    # 차선 중앙에서 얼마나 떨어져 있는지를 이용한 PD제어
    Steer_Configuration()
    PD_Setup()
    Line_Configuration()
    PD_Control()

    #=========================================
    # 차량의 속도 값인 speed값 정하기.
    # 직선 코스에서는 빠른 속도로 주행하고 
    # 회전구간에서는 느린 속도로 주행하도록 설정함.
    #=========================================

    # 차량의 속력을 제어하는 코드. 직선과 곡선에서 속력을 다르게 입력한다.
    if(Rubber.start_avoid_flag == False):

        if curve_flag:
            if Rubber.right_state and left_flag: #흰 차선이 바깥쪽이라면 속도를 더 낼수 있음 !!
                speed = TURN_FASTVEL
            
            elif Rubber.left_state and right_flag:
                speed = TURN_FASTVEL

            else:
                speed = turn_vel
        
        else:
            speed = straight_vel
    else:
        speed = avoid_speed


    # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
    # print(angle,speed)
    drive(angle, speed)

#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================
def start():
    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    # 전역 변수로 사용하기 위해 global 선언해 준다.
    global motor, image
    global turn_vel, straight_vel
    global driving_flag
    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    rubber_sub = rospy.Subscriber("/scan", LaserScan, Rubber.cb_scan)
    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        print("[Waiting for image...] current size:", image.size)
        time.sleep(0.1)


    while not driving_flag : #녹색 신호를 받을 때 까지 대기
        driving_flag = 1#sinho_detect(image)
        #print(driving_flag) 
        time.sleep(0.1)
    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown(): #라인 다 통과했으면 탈출하자.
        
        if ((not Rubber.rubber_flag) and (not Rubber.lane_change_flag)) :
            
            lane_follow(0) #러버 보기 전까지 차선따라가기..
        
        if Rubber.rubber_ready_flag :
            turn_vel = 30
            straight_vel = 30
        
        if Rubber.end_flag:
            turn_vel = TURN_VELO
            straight_vel = STRAIGHT_VELO
            Rubber.rubber_flag = False
            
        #print(Rubber.end_flag)
        ##################################################여기부터 차량 회피 구간 !!#########################################################
        # 전방 차량 감지 시 상대속도 계산 시작
        if(Rubber.lane_change_flag == True):
            if (Rubber.start_avoid_flag == False): #장애물을 회피하지 않을 때에는 차선인식.
                #print("!!!!!")
                current_speed = avoid.cal_car_vel(Rubber)
                #print(current_speed)
                lane_follow(current_speed)
            
            # 차량 회피 플레그 들어오면 틀어서 회피주행 시작
            if((Rubber.start_avoid_flag == True)and(Rubber.right_state == True)and(Rubber.go_back_flag == False)):
                print("move to L")
                drive(-AVOID_ANGLE,AVOID_VEL)
                time.sleep(0.1)

            elif((Rubber.start_avoid_flag == True)and(Rubber.left_state == True)and(Rubber.go_back_flag == False)):
                print("move to R")
                drive(AVOID_ANGLE,AVOID_VEL)
                time.sleep(0.1)
            
            if(Rubber.go_back_flag == True):
                lane_follow(0)
                # if((Rubber.start_avoid_flag == True)and(Rubber.right_state == True) and (any(x < 85 for x in Rubber.ranges[75:140]))):
                #     Rubber.lane_change_flag = True
                #     Rubber.left_state = True
                #     Rubber.right_state = False
                # elif((Rubber.start_avoid_flag == True)and(Rubber.left_state == True) and (any(x < 85 for x in Rubber.ranges[40:105]))):
                #     Rubber.lane_change_flag = True
                #     Rubber.left_state = False
                #     Rubber.right_state = True
                # else:
                Rubber.lane_change_flag = False
                Rubber.start_avoid_flag = False #원상복구
                Rubber.go_back_flag = False #원상복구
                Rubber.left_state = False
                Rubber.right_state = False
                time.sleep(0.1)

                

        


    # while not rospy.is_shutdown(): #차량 만날때까지 차선따라가기.
    #     lane_follow()
        
    #     #avoid_wd_lidar.lane_change_trigger(avoid_wd_lidar.ranges) #라이다 값으로 차량 회피 트리거


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()