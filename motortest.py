#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
import time
import signal
import sys
import os
import math

import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu # 나중에 쓸 imu센서 massage import
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
#from curve_navigator import CurveNavigator  # 클래스 import
#navigator = CurveNavigator()    # 여기서 self가 navigator라는 이름으로 매핑됨

signal.signal(signal.SIGINT, lambda sig, frame: (time.sleep(3), os.system('killall -9 python rosout'), sys.exit(0)))
look_ahead = 1.0
image = None
bridge = CvBridge()
motor = None
lx, ly, rx, ry = [], [], [], []
angle, degree, error, DX, DY = 0, 0, 0, 0, 0
left_flag, right_flag = 0, 0
left_state, right_state = 0, 0
prev_time = 0
prev_angle = 0
kernal = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))

# 주행 모드 (0: 중앙차선, 1: 한차선)
driving_mode = 0
 
speed = 0.0
# 설정값

straight_falg = False
Turn_flag = True

WIDTH, HEIGHT = 640, 480
LANE_WIDTH = 415
CAM_FPS = 30
START_BOX, TARGET_BOX = 1, 7
MAX_DEGREE = 50
MAX_ERROR = 50
DX_GAIN, DEGREE_GAIN = 0.2, 2.3
#STRAIGHT_VELO, TURN_VELO = 100, 40
STRAIGHT_VELO, TURN_VELO = 7, 7
TURN_FASTVEL = TURN_VELO #+ 30
nwindows, margin, minpix, lane_bin_th = 8, 50, 5, 80
KERNEL_SIZE = 5
CONFIG_DEGREE = 3

# HSV
LOW_YELLOW, HIGH_YELLOW = np.array([20, 100, 100]), np.array([30, 255, 255])
LOW_WHITE, HIGH_WHITE = np.array([0, 0, 190]), np.array([180, 25, 255])

# 버드아이뷰
#SOURCE_POINTS_CENTER = np.float32([[65, 150], [5, 175], [252.5, 150], [312.5, 175]])
SOURCE_POINTS_CENTER = np.float32([[110, 260], [37, 300], [520, 260], [605, 300]]) # 가까이
#SOURCE_POINTS_CENTER = np.float32([[189, 220], [98, 260], [422, 220], [516, 260]]) # 소스 포인트 수정08.06(middle mode) 멀리
DESTINATION_POINTS_CENTER = np.float32([[98, 10], [98, 470], [538, 10], [538, 470]]) #모아서
SOURCE_POINTS_SINGLE = np.float32([[105, 150], [20, 219], [212, 150], [297, 226]]) #벌려서
DESTINATION_POINTS_SINGLE = np.float32([[160, 10], [160, 470], [480, 10], [480, 470]])


DESTINATION_POINTS = np.float32([[160, 10], [160, 470], [480, 10], [480, 470]])
window_height = int(HEIGHT / nwindows)


def homomorphic_filter(gray, gammaL=0.6, gammaH=1.8, c=1.0, D0=30, use_butterworth=True, n=2):
    """
    gray: uint8 그레이스케일 이미지
    gammaL: 저주파(조명) 억제 계수(작을수록 더 조명,그림자 억제) 0.3~0.8
    gammaH: 고주파(디테일) 증폭 계수(클수록 더 경계 강조) 1.2~2.5
    c: 전이 경사(클수록 경계 급함. 칼같이 끊기면서 날카롭게.) 0.5~2.0
    D0: 컷오프 주파수(픽셀). 작으면 강한 억제/강조, 크면 완만하고 부드러움.
    use_butterworth: 가우시안 대신 버터워스 전이 사용 여부
    n: 버터워스 차수(2~4 추천) 차수가 클수록 칼같이 끊긴다.
    """
    if len(gray.shape) != 2:
        raise ValueError("gray must be single-channel")

    # float 변환 + 로그
    f = gray.astype(np.float32) / 255.0
    f_log = np.log1p(f)

    # FFT
    F = np.fft.fftshift(np.fft.fft2(f_log))

    # 필터 H(u,v) 만들기
    rows, cols = gray.shape
    u = np.arange(rows) - rows/2
    v = np.arange(cols) - cols/2
    V, U = np.meshgrid(v, u)  # 주의: (행=y, 열=x)
    D2 = (U**2 + V**2)

    if use_butterworth:
        # 고역통과 버터워스 형태로 가중(하이부스트): H = (γH-γL)*(1 - 1 / (1 + (D0^2/D^2)^{n/2})) + γL
        # D=0에서 분모 0 방지용 eps
        H_hp = 1.0 - 1.0 / (1.0 + ( (D2 / (D0**2)) ** n ))
        H = (gammaH - gammaL) * H_hp + gammaL
    else:
        # 가우시안 기반 전형적 호모모픽: H = (γH-γL)*(1 - exp(-c * D^2 / D0^2)) + γL
        H = (gammaH - gammaL) * (1 - np.exp(-c * D2 / (D0**2))) + gammaL

    # 필터 적용
    G = H * F

    # 역FFT + exp 복원
    g_log = np.fft.ifft2(np.fft.ifftshift(G))
    g = np.expm1(np.real(g_log))

    # 정규화
    g = np.clip(g, 0, None)
    g = g / (g.max() + 1e-8)
    out = (g * 255).astype(np.uint8)
    return out


def histogram(lane):
    global lx, ly, rx, ry, left_state, right_state
    hist = np.sum(lane[lane.shape[0]//2:, :], axis=0)
    mid = hist.shape[0] // 2
    lx_cur = np.argmax(hist[:mid])
    rx_cur = np.argmax(hist[mid:]) + mid
    nz = lane.nonzero()
    lx, ly, rx, ry = [], [], [], []
    l_count, r_count = 0, 0
    left_lane_inds = []
    right_lane_inds = []
    out_img = np.dstack((lane,lane,lane))*255


    for w in range(nwindows):
        win_y_low = lane.shape[0] - (w + 1) * window_height
        win_y_high = lane.shape[0] - w * window_height
        win_xll, win_xlh = lx_cur - margin, lx_cur + margin
        win_xrl, win_xrh = rx_cur - margin, rx_cur + margin
        cv2.rectangle(out_img,(win_xll,win_y_low),(win_xlh,win_y_high),(0,255,0),2)
        cv2.rectangle(out_img,(win_xrl,win_y_low),(win_xrh,win_y_high),(0,255,0),2)
        l_inds = ((nz[0] >= win_y_low) & (nz[0] < win_y_high) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        r_inds = ((nz[0] >= win_y_low) & (nz[0] < win_y_high) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]
        left_lane_inds.append(l_inds)
        right_lane_inds.append(r_inds)
        if len(l_inds) > minpix:
            lx_cur = int(np.mean(nz[1][l_inds]))
            l_count += 1
        if len(r_inds) > minpix:
            rx_cur = int(np.mean(nz[1][r_inds]))
            r_count += 1
        lx.append(lx_cur)
        ly.append((win_y_low + win_y_high) / 2)
        rx.append(rx_cur)
        ry.append((win_y_low + win_y_high) / 2)
    left_state = int(l_count > r_count)
    right_state = int(r_count > l_count)
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    out_img[nz[0][left_lane_inds],nz[1][left_lane_inds]]=[255,0,0]
    out_img[nz[0][right_lane_inds],nz[1][right_lane_inds]]=[0,0,255]
    cv2.imshow("viewer", out_img)


def Steer_Configuration():
    global lx, rx
    if (sum(lx) / len(lx)) < 30: #왼쪽 차선의 평균 좌표가 30 이하라면 0으로 박음
        lx = [0] * len(lx)
    elif (sum(rx) / len(rx)) > (WIDTH - 40): #오른쪽 차선의 평균 좌표가 600 이상이이라면 최대 넓이로 박음
        rx = [WIDTH] * len(rx)
    empty_line_flag = 1 if sum(lx) == 0 or sum(rx)/len(rx) == WIDTH else 0 #왼쪽 차선 좌표가 0이거나, 오른쪽 차선 좌표가 넓이라면. -> 둘중 하나를 놓쳤을 때
    fusion_line_flag = 1 if lx == rx else 0 # 두 차선이 동일하다면.
    #print(empty_line_flag)
    #print(sum(lx) / len(lx))
    #print(sum(rx) / len(rx))
    if empty_line_flag:
        if sum(rx) and (sum(rx)/len(rx)) > (WIDTH/2):
            lx[TARGET_BOX] = rx[TARGET_BOX] - LANE_WIDTH
            lx[START_BOX] = rx[START_BOX] - LANE_WIDTH
        elif sum(lx) and (sum(lx)/len(lx)) < (WIDTH/2):
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH
        elif (sum(lx)/len(lx)) == 0 and (sum(rx)/len(rx)) == WIDTH:
            lx = [WIDTH//4] * len(lx)
            rx = [WIDTH//4*3] * len(rx)
    if fusion_line_flag:
        if rx[TARGET_BOX] < WIDTH//2 + 30:
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH
        elif lx[TARGET_BOX] > WIDTH//2 - 30:
            lx[TARGET_BOX] = rx[TARGET_BOX] - LANE_WIDTH
            lx[START_BOX] = rx[START_BOX] - LANE_WIDTH
    if (rx[START_BOX] - lx[START_BOX]) < (LANE_WIDTH * 0.8):
        if ((rx[START_BOX] + lx[START_BOX]) / 2) < (WIDTH / 2):
            rx[TARGET_BOX] = lx[TARGET_BOX] + LANE_WIDTH
            rx[START_BOX] = lx[START_BOX] + LANE_WIDTH
        elif ((rx[START_BOX] + lx[START_BOX]) / 2) > (WIDTH / 2):
            lx[TARGET_BOX] = rx[TARGET_BOX] - LANE_WIDTH
            lx[START_BOX] = rx[START_BOX] - LANE_WIDTH

def PD_Setup():
    global degree, error, DX, DY
    print(rx[TARGET_BOX],lx[TARGET_BOX])
    print(rx[START_BOX] ,lx[START_BOX])
    DX = ((rx[TARGET_BOX] + lx[TARGET_BOX]) / 2) - ((rx[START_BOX] + lx[START_BOX]) / 2)
    #DX가 음수라면 좌회전, 양수라면 우회전.
    DY = ((ry[START_BOX] + ly[START_BOX]) / 2) - ((ry[TARGET_BOX] + ly[TARGET_BOX]) / 2)
    degree = (math.atan2(DX, DY) * 180) / math.pi if DY else 0
    error = (rx[TARGET_BOX] + lx[TARGET_BOX] + rx[START_BOX] + lx[START_BOX]) / 4 - (WIDTH / 2) 
    #전체적인 차선이 왼쪽에 있다면 (차량이 오른쪽에 존재한다면 )에러가 음수, 반대라면 에러가 양수
    if abs(error) > MAX_ERROR:
        error = MAX_ERROR if error > 0 else -MAX_ERROR

    if abs(degree) > MAX_DEGREE:
        degree = MAX_DEGREE if degree > 0 else -MAX_DEGREE


def steer_Control():
    global angle, look_ahead, prev_angle,straight_falg,Turn_flag
    ym = (lx[TARGET_BOX]+rx[TARGET_BOX]) / 2 - WIDTH / 2
    #print("ym ",ym)
    xt = look_ahead

    steer = np.arctan2(ym, xt)
    angle = np.degrees(steer) * DEGREE_GAIN

    angle = 0.7*prev_angle + 0.3*angle
    prev_angle = angle
    angle = np.clip(angle, -MAX_DEGREE, MAX_DEGREE)
    if abs(degree) < CONFIG_DEGREE:
        straight_falg = True
        Turn_flag = False
        
        angle = angle*0.3
    
    else:
        straight_falg = False
        Turn_flag = True

def PD_Control():
    global angle, left_flag, right_flag, prev_angle,straight_falg,Turn_flag
    right_flag = 1 if DX > 0 else 0
    left_flag = 1 if DX < 0 else 0
    # PD 계산
    angle = error * DX_GAIN + degree * DEGREE_GAIN

    angle = 0.7*prev_angle + 0.3*angle
    prev_angle = angle

    # 좌/우 방향 부호 붙여줌
    #if left_flag:
    #    angle = -abs(angle)
    #else:
    #    angle = abs(angle)

    # 최종 조향 각도 제한
    angle = np.clip(angle, -MAX_DEGREE, MAX_DEGREE)

    if abs(degree) < CONFIG_DEGREE:
        straight_falg = True
        Turn_flag = False
        
        #angle = angle*0.3
    
    else:
        straight_falg = False
        Turn_flag = True



def lane_follow():
    global image, driving_mode, angle, speed, node,kernal, straight_falg, Turn_flag
    raw = image.copy() #hsv용
    raw2 = image.copy() #호모모픽용
    raw3 = image.copy() #쌩이미지보기


#############hsv
    hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, LOW_WHITE, HIGH_WHITE)
    result = cv2.bitwise_and(raw, raw, mask=mask)
##############

##################버드아이뷰 표시용
    points = [[110, 260], [37, 300], [520, 260], [605, 300]]
    for pt in points:
        cv2.circle(raw3,pt,radius=5,color=(0,0,255),thickness=-1)
    cv2.imshow("BIR", raw3)
#########################################


    M = cv2.getPerspectiveTransform(SOURCE_POINTS_CENTER, DESTINATION_POINTS_CENTER)
    bird = cv2.warpPerspective(result, M, (WIDTH, HEIGHT))
    #cv2.imshow("lfffffff", bird)

    # Gaussian Blur를 사용하여 이미지의 노이즈 제어
    #blur_img = cv2.GaussianBlur(bird, (5, 5), 0)
    L = cv2.cvtColor(bird, cv2.COLOR_BGR2HLS)[:, :, 1]
    #hom = homomorphic_filter(L, gammaL=0.9, gammaH=2.5, c=1.8, D0=15)
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY)
    #opened = cv2.morphologyEx(lane,cv2.MORPH_OPEN,kernal)
    closed = cv2.morphologyEx(lane,cv2.MORPH_CLOSE,kernal)
    #cv2.imshow("o1111111111", closed)
    # DK -> 150, ku -> 60 ~ 80
    #_, lane = cv2.threshold(closed, lane_bin_th, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)       #이미지에서 찾은 윤곽선(컨투어)들의 각 꼭짓점 배열의 리스트를 저장  
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)  # 해당 컨투어의 컨투어(cnt)**를 감싸는 최소 크기의 직사각형을 구하고 - 왼쪽 위 꼭짓점(x,y),높이,너비 반환
        area = cv2.contourArea(cnt)     # 해당 컨투어의 면적(픽셀수) 구함

        mask = np.zeros_like(closed)      # 이전에 이진화로 합친 이미지(lane)의 화면 크기인데 다 0으로 채워진거 하나 만듬
        cv2.drawContours(mask, [cnt], -1, 255, -1)    # 그 0으로 채워진 검정 화면에 해당 컨투어만 흰색으로 그림 --> 즉 해당 컨투어 부분만 뽑힌다
        roi = cv2.bitwise_and(closed, mask)          # 이진화된 lane 이미지와 해당 컨투어만 뽑아낸걸 and연산해서 컨투어 부분만 뽑는다

        vertical_profile = np.sum(roi, axis=1) // 255          # 행 방향(y축)**으로 픽셀 값을 전부 더함 → 즉, 각 행마다 흰색 픽셀이 얼마나 있는지 계산/ 그걸 255로 나눌시 각행의  흰색 픽셀 개수가 나옴 / 따라서  이미지의 각 행마다 흰색 픽셀(=특징점)이 몇 개인지를 담은 1차원 배열 vertical_profile 얻음.
                                                                #컨투어로 뽑힌곳에서만

        consecutive_white_rows = np.count_nonzero(vertical_profile > 15)  # 해당 컨투어의 각행에서 흰색픽셀이 10개 이상인 경우만 consecutive_white_rows 에저장 

        if consecutive_white_rows > 30:         # 해당 컨투어에서 흰색픽셀이 10개 이상인게 30행이 넘는다면(세로로 충분이 기냐(차선처럼)? 판단)  ***************************
            continue                                    # 이건 차선이다 인식하고 다음 컨투어로 검사

        if area < 1800:                                 # 혹시 30행이 안넘는데(세로로 길지 않는데) 컨투어 내부 픽셀 면적이 800 이하면 (길이가 길지 않다면(곡선의 경우) 그 컨투어의 면적이 충분히 의미있는 크기냐?)
            cv2.drawContours(closed, [cnt], -1, 0, -1)   # 차선처럼 세로로 길지도않고, 안긴데 면적이 유의미할만큼 안크면 이진화 lane이미지에서 해당 컨투어로 감싼 영역을 다 0으로 한다
        else:
            cv2.drawContours(closed, [cnt], -1, 255, -1)

    cv2.imshow("hsv_result", closed)




############여기부터 호모모픽
    bird2 = cv2.warpPerspective(raw2, M, (WIDTH, HEIGHT),
                            flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_REPLICATE)
    L2 = cv2.cvtColor(bird2, cv2.COLOR_BGR2HLS)[:, :, 1]
    hom = homomorphic_filter(L2, gammaL=0.3, gammaH=2.3, c=1.8, D0=15)
    #cv2.imshow("hmhohmhomh", hom)
    _, lane2 = cv2.threshold(hom, lane_bin_th, 255, cv2.THRESH_BINARY)
    #cv2.imshow("2222222", lane2)
    closed2 = cv2.morphologyEx(lane2,cv2.MORPH_CLOSE,kernal)
    #cv2.imshow("mopopopop", closed2)
    contours2, _ = cv2.findContours(closed2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)       #이미지에서 찾은 윤곽선(컨투어)들의 각 꼭짓점 배열의 리스트를 저장  
    for cnt in contours2:
        x, y, w, h = cv2.boundingRect(cnt)  # 해당 컨투어의 컨투어(cnt)**를 감싸는 최소 크기의 직사각형을 구하고 - 왼쪽 위 꼭짓점(x,y),높이,너비 반환
        area = cv2.contourArea(cnt)     # 해당 컨투어의 면적(픽셀수) 구함

        mask = np.zeros_like(closed)      # 이전에 이진화로 합친 이미지(lane)의 화면 크기인데 다 0으로 채워진거 하나 만듬
        cv2.drawContours(mask, [cnt], -1, 255, -1)    # 그 0으로 채워진 검정 화면에 해당 컨투어만 흰색으로 그림 --> 즉 해당 컨투어 부분만 뽑힌다
        roi = cv2.bitwise_and(closed, mask)          # 이진화된 lane 이미지와 해당 컨투어만 뽑아낸걸 and연산해서 컨투어 부분만 뽑는다

        vertical_profile = np.sum(roi, axis=1) // 255          # 행 방향(y축)**으로 픽셀 값을 전부 더함 → 즉, 각 행마다 흰색 픽셀이 얼마나 있는지 계산/ 그걸 255로 나눌시 각행의  흰색 픽셀 개수가 나옴 / 따라서  이미지의 각 행마다 흰색 픽셀(=특징점)이 몇 개인지를 담은 1차원 배열 vertical_profile 얻음.
                                                                #컨투어로 뽑힌곳에서만

        consecutive_white_rows = np.count_nonzero(vertical_profile > 15)  # 해당 컨투어의 각행에서 흰색픽셀이 10개 이상인 경우만 consecutive_white_rows 에저장 

        if consecutive_white_rows > 30:         # 해당 컨투어에서 흰색픽셀이 10개 이상인게 30행이 넘는다면(세로로 충분이 기냐(차선처럼)? 판단)  ***************************
            continue                                    # 이건 차선이다 인식하고 다음 컨투어로 검사

        if area < 1800:                                 # 혹시 30행이 안넘는데(세로로 길지 않는데) 컨투어 내부 픽셀 면적이 800 이하면 (길이가 길지 않다면(곡선의 경우) 그 컨투어의 면적이 충분히 의미있는 크기냐?)
            cv2.drawContours(closed2, [cnt], -1, 0, -1)   # 차선처럼 세로로 길지도않고, 안긴데 면적이 유의미할만큼 안크면 이진화 lane이미지에서 해당 컨투어로 감싼 영역을 다 0으로 한다
        else:
            cv2.drawContours(closed2, [cnt], -1, 255, -1)

    cv2.imshow("homo_result", closed2)



    real = cv2.bitwise_and(closed, closed2)
    cv2.imshow("REALLLLLLL", real)


    #cv2.imshow("hsv", result)
    #cv2.imshow("oc222222", closed)
    #cv2.imshow("lane", lane)
    cv2.waitKey(1)
    histogram(real)
    Steer_Configuration()
    PD_Setup()
    PD_Control()
    #speed = STRAIGHT_VELO if abs(degree) < CONFIG_DEGREE else (TURN_FASTVEL if (right_state and left_flag) or (left_state and right_flag) else TURN_VELO)
    speed = STRAIGHT_VELO if straight_falg else TURN_VELO
    #("straignt_flag",straight_falg)
    #print("angle\n", angle)
    
    #print("degree\n", degree)
    print("error\n", error)
    #print("speed\n", speed)
    #print(lx[START_BOX],rx[START_BOX],lx[TARGET_BOX],rx[TARGET_BOX])
    #print("speed\n", speed)
    node.drive(angle, speed)

# 메인 루프

class DriverNode(Node):
    def __init__(self):

        global driving_mode, bridge, image
        super().__init__('driver')
        self.motor_publisher = self.create_publisher(Float32MultiArray, 'xycar_motor', 1)
        self.motor_msg = Float32MultiArray()

        # 파라미터 초기화
        self.speed = self.declare_parameter("speed", 0).value
        self.angle = 0
        self.delta = 10
        self.get_logger().info('----- Xycar self-driving node started -----')

        
        # CvBridge 객체 초기화
        bridge = CvBridge()
        
        # 이미지를 저장할 변수 초기화
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.img_callback,
            10  # 큐 크기 설정
        )

        # 카메라 토픽 도착 대기 (여기서 블로킹)
        while image is None and rclpy.ok():
            self.get_logger().info("Waiting for Camera image...")
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)  # CPU 사용률 최적화

        self.get_logger().info("Camera Ready --------------")

        os.system('v4l2-ctl -d /dev/videoCAM -c auto_exposure=1')
        os.system(f'v4l2-ctl -d /dev/videoCAM -c exposure_time_absolute=110') #노출도 설정

    def img_callback(self,data):
        global image, bridge
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        # image = cv2.resize(image, (WIDTH, HEIGHT))


    def drive(self, angle, speed): 
        self.motor_msg.data = [float(angle), float(speed)] 
        self.motor_publisher.publish(self.motor_msg)

    def run(self):
        try:
            while rclpy.ok():
                #plt.pause(0.001)
                rclpy.spin_once(self, timeout_sec=0.1)

                lane_follow()

                #if driving_mode == 0:
                    #print("CENTER DRIVING")
                #else:
                    #print("ONE DRIVING")

        except KeyboardInterrupt:
            pass


def start():
    global motor, image, driving_mode, node
    rclpy.init()
    node = DriverNode()
    node.run() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    start()

