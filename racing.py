#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2
import rospy
import time
import signal
import sys
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import XycarMotor
from sinhodeng import sinho_detect  # 신호등 감지 함수 받아옴
import math

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
bridge = CvBridge()
motor = None  # 모터 토픽을 담을 변수
lx, ly, rx, ry = [], [], [], []
left_state, right_state = 0, 0
prev_time = 0
#=============================================
# 상수 선언부
#=============================================
KERNEL_SIZE = 5

# 노란색·흰색 차선 검출 HSV 범위
LOW_YELLOW = np.array([20, 100, 100])
HIGH_YELLOW = np.array([30, 255, 255])
LOW_WHITE  = np.array([0, 0,  50])
HIGH_WHITE = np.array([0, 0, 255])

# SLIDING WINDOW용 상수
nwindows = 8
margin    = 40
minpix    = 5
lane_bin_th = 80

# PD 제어용 상수
START_BOX     = 1
TARGET_BOX    = 7
MAX_DEGREE    = 50
DX_GAIN       = 0.27
DEGREE_GAIN   = 0.203
CONFIG_DEGREE = 1.0

# 속도 설정
STRAIGHT_VELO = 100
TURN_VELO     = 100
TURN_FASTVEL  = TURN_VELO + 30

# 카메라 및 이미지 크기
CAM_FPS = 30
WIDTH, HEIGHT = 320, 240

# 원근 변환 행렬 미리 계산
SOURCE_POINTS = np.float32([
    [65, 150],     # L_UP
    [5, 175],      # L_DOWN
    [252.5, 150],  # R_UP
    [312.5, 175]   # R_DOWN
])

DESTINATION_POINTS = np.float32([
    [86, 5],       # L_UP → 상단 중심 좌표로 이동
    [80, 235],     # L_DOWN
    [236, 5],      # R_UP
    [240, 235]     # R_DOWN
])


TRANSFORM_MATRIX   = cv2.getPerspectiveTransform(SOURCE_POINTS, DESTINATION_POINTS)
window_height      = int(HEIGHT / nwindows)
LANE_WIDTH = 140		    # BIRD_EYE_VIEW 를 통해 변환된 차선의 폭 값인 상수값을 저장하는 변수.
#=============================================
# 카메라 토픽 콜백 함수
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    image = cv2.resize(image, (320, 240))

#=============================================
# 히스토그램 기반 차선 검출
#=============================================
def histogram(lane):
    global lx, ly, rx, ry, left_state, right_state

    hist = np.sum(lane[lane.shape[0]//2:, :], axis=0)
    midpoint = int(hist.shape[0] / 2)
    leftx_current  = np.argmax(hist[:midpoint])
    rightx_current = np.argmax(hist[midpoint:]) + midpoint

    nz = lane.nonzero()
    lx, ly, rx, ry = [], [], [], []
    left_lane_inds = []
    right_lane_inds = []
    l_count, r_count = 0, 0 # 현재 왼쪽인지 오른쪽인지 구분하기 위한 변수.
    pre_leftx, pre_rightx = 0, 0 
    dxl, dxr = 0, 0
    color_out = np.dstack((lane, lane, lane)) * 255

    
    #t0 = time.perf_counter()


    for window in (range(nwindows)):
        win_y_low = lane.shape[0] - (window+1) * window_height
        win_y_high = lane.shape[0] - window * window_height

        win_xll = leftx_current  - margin
        win_xlh = leftx_current  + margin
        win_xrl = rightx_current - margin
        win_xrh = rightx_current + margin

        cv2.rectangle(color_out, (win_xll, win_y_low),(win_xlh, win_y_high),(0,255,0),2) 
        cv2.rectangle(color_out, (win_xrl, win_y_low),(win_xrh, win_y_high),(0,255,0),2)

        good_left_inds  = ((nz[0]>=win_y_low)&(nz[0]<win_y_high)&(nz[1]>=win_xll)&(nz[1]<win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0]>=win_y_low)&(nz[0]<win_y_high)&(nz[1]>=win_xrl)&(nz[1]<win_xrh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nz[1][good_left_inds]))


        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nz[1][good_right_inds]))

            

        lx.append(leftx_current)
        ly.append((win_y_low + win_y_high)/2)
        rx.append(rightx_current)
        ry.append((win_y_low + win_y_high)/2)
        #print(lx,ly,rx,ry)


    #t1 = time.perf_counter()
    #print(f" Histogram: {(t1-t0)*1000:.2f} ms")
    # 시각화

    # 배열을 1차원으로 합침
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    #print (lx,rx)
    

    color_out[nz[0][left_lane_inds], nz[1][left_lane_inds]]   = [255,0,0]
    color_out[nz[0][right_lane_inds],nz[1][right_lane_inds]]   = [0,0,255]
    cv2.imshow("viewer", color_out)

    
    


#=============================================
# 차선 기울기·조향 제어 설정
#=============================================
def Turn_Configuration():
    pass  # 함수 내부 로직이 필요 없으므로 패스

def Steer_Configuration():
    global lx, rx
    # 오작동 시 기본값 보정
    if (sum(lx)/len(lx)) < 30:
        lx = [0] * len(lx)
    elif (sum(rx)/len(rx)) > (WIDTH - 40):
        rx = [WIDTH] * len(rx)

    # 차선의 없음을 인식하는 코드. 왼쪽 차선의 x 값들의 합이 0 또는 오른쪽 차선의 x 값들의 합이 640인 경우에 empty_line_flag = ON.
    empty_line_flag = 1 if sum(lx) == 0  or sum(rx)/len(rx) == (WIDTH) else 0
    
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

        # 왼쪽 차선이 인식되어 있고, 왼쪽 차선의 x 값이 화면 중앙의 값보다 작으면 실행. 국민대 대회에서는 이곳에 들어올 일이 없다.. AMET을 위해서는 좀 손봐야할듯.
        elif sum(lx) and (sum(lx) / len(lx)) < (WIDTH/2) : 
            # 오른쪽 차선의 x 값을 왼쪽 차선 x 값에서 차선폭 길이만큼 더해줌. 
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH 

        # 차선이 모두 인식되지 않는 상황.
        elif (sum(lx)/len(lx)) == 0  and ((sum(rx)/len(rx)) == (WIDTH)) : 
            # 왼쪽 차선과 오른쪽 차선의 값을 조정해 직선으로 주행하도록 함 
            lx = [160] * len(lx)
            rx = [480] * len(rx)
    # 두 선이 중복 출력될 경우  뭔가 중앙에서 선이 잡혀서 애매하게 판단될떄 인듯 .. 만약 오른쪽이 
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
    # 겹치는 상황이나 한쪽만 인식한 상황에서는 아래의 분기에 안들어간다. 이미 새로 라인을 그렸기 때문 ..
    # 인식된 두 차선간 거리가 LANE_WIDTH * 0.8 보다 작은 경우
    
    if (rx[START_BOX] - lx[START_BOX]) < (LANE_WIDTH * 0.8) :
        
		# 사각형 먼쪽의 x의 평균값이 화면 넓이의 절반보다 작은 경우 실행.
        if ((rx[START_BOX] + lx[START_BOX]) / 2) < (WIDTH / 2) :
            # 왼쪽 차선의 x 값에 차선 폭만큼 더해 오른쪽 차선의 x 값에 대입.
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH
            #print("L")

        # 사각형 먼쪽의 x의 평균갑이 화면 넓이의 절반보다 큰 경우 실행.
        elif ((rx[START_BOX] + lx[START_BOX]) / 2) > (WIDTH / 2) :
            # 왼쪽 차선의 x 값에 차선 폭만큼 더해 오른쪽 차선의 x 값에 대입.
            lx[TARGET_BOX] = rx[TARGET_BOX] - LANE_WIDTH
            lx[START_BOX] = rx[START_BOX] - LANE_WIDTH
            #print("R")

    else : 
        pass
    #print(lx,rx)
#=============================================
# PD 제어 계산
#=============================================
def PD_Setup():
    global degree, error, DX, DY
    DX = ((rx[TARGET_BOX] + lx[TARGET_BOX]) / 2) - ((rx[START_BOX] + lx[START_BOX]) / 2)
    DY = ((ly[START_BOX] + ry[START_BOX]) / 2) - ((ly[TARGET_BOX] + ry[TARGET_BOX]) / 2)
    degree = (math.atan2(DX, DY) * 180) / math.pi if DY else 0
    error = (WIDTH/2) - (rx[TARGET_BOX] + lx[TARGET_BOX] + rx[START_BOX] + lx[START_BOX]) / 4
    if abs(degree) > MAX_DEGREE:
        degree = MAX_DEGREE * (1 if degree>0 else -1)

#=============================================
# 직선 vs 곡선 판단
#=============================================
def Line_Configuration():
    global straight_flag, curve_flag
    if abs(degree) < CONFIG_DEGREE:
        straight_flag, curve_flag = 1, 0
    else:
        straight_flag, curve_flag = 0, 1

#=============================================
# PID 기반 조향 각도 계산
#=============================================
def PD_Control():
    global angle, right_flag, left_flag
    right_flag = 1 if DX>0 else 0
    left_flag  = 1 if DX<0 else 0
    angle = abs(error * DX_GAIN + degree * DEGREE_GAIN)
    angle = -angle if left_flag else angle

#=============================================
# 모터 토픽 발행
#=============================================
def drive(angle, speed):
    global motor
    msg = XycarMotor()
    msg.angle = angle*3
    msg.speed = speed
    motor.publish(msg)

#=============================================
# 차선 따라가기 메인 로직
#=============================================
def lane_follow():
    global left_state, right_state, left_flag, right_flag, prev_time
    raw_img = image.copy()

    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time

    hsv = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV)
    #mask_y = cv2.inRange(hsv, LOW_YELLOW, HIGH_YELLOW)
    mask_w = cv2.inRange(hsv, LOW_WHITE, HIGH_WHITE)
    #mask = cv2.bitwise_or(mask_y, mask_w)
    result = cv2.bitwise_and(raw_img, raw_img, mask=mask_w)


    pts = SOURCE_POINTS.reshape((-1, 1, 2)).astype(np.int32)
    cv2.polylines(raw_img, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

    cv2.putText(raw_img, f"FPS: {fps:.2f}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("VViewer",raw_img)
    
    bird_view = cv2.warpPerspective(result, TRANSFORM_MATRIX, (WIDTH, HEIGHT)) #WSL 기준 0.8ms 걸림..

    L = cv2.cvtColor(bird_view, cv2.COLOR_BGR2HLS)[:,:,1]
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY)
    cv2.waitKey(1)

    histogram(lane)
    Turn_Configuration()
    Steer_Configuration()
    PD_Setup()
    Line_Configuration()
    PD_Control()

    if curve_flag:
        if right_state and left_flag: #흰 차선이 바깥쪽이라면 속도를 더 낼수 있음 !!
            speed = TURN_FASTVEL
        
        elif left_state and right_flag:
            speed = TURN_FASTVEL

        else:
            speed = TURN_VELO
    
    else:
        speed = STRAIGHT_VELO
    drive(angle, speed)

#=============================================
# 메인 함수
#=============================================
def start():
    global motor, image
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback)

    print('----- Xycar self driving -----')
    while not image.size == (WIDTH*HEIGHT*3):
        print('[Waiting for image...] current size:', image.size)
        time.sleep(0.1)

    driving_flag = 0
    while not driving_flag:
        driving_flag = sinho_detect(image)
        print(driving_flag)
        time.sleep(0.1)

    while not rospy.is_shutdown():
        lane_follow()

if __name__ == '__main__':
    start()
