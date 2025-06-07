#!/usr/bin/env python3
# coding: utf-8
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import XycarMotor
import matplotlib.pyplot as plt
import avoid_wd_lidar as avoid
import time

drive_fn = None


class CurveNavigator:
    def __init__(self):
        #rospy.init_node('curve_navigator', anonymous=True)
        plt.ion()                          # interactive mode on
        self.fig, self.ax = plt.subplots() #전체 그림 객체와 그래프 개체
        self.sub = rospy.Subscriber("/scan", LaserScan, self.cb_scan)
        self.pub = rospy.Publisher("xycar_motor", XycarMotor, queue_size=1)
        self.motor = XycarMotor()

        # 피팅 파라미터
        self.right_sector = (1, 89)    # 45±30
        self.left_sector  = (271, 359)  # 315±30
        self.fit_deg     = 1
        self.look_ahead  = 1.1
        self.K           = 1.8
        self.lane_width  = 0.6
        self.left_line_flag = False
        self.right_line_flag = False
        self.xl,self.yl,self.xr,self.yr = [] , [] , [] , []
        self.rubber_flag = False
        self.end_flag = False
        self.rubber_ready_flag = False

        self.lane_change_flag = False # 차선 변경 함수 돌리기 시작하기 위한 flag
        self.right_state = 0 # 자신이 위치한 차선 판단
        self.left_state = 0 # 자신이 위치한 차선 판단
        self.front_car_vel = 0 # 상대 속도
        self.front_car_dis = 0 # 상대 거리
        self.side_car_flag = 0 # 추월 시 차량 옆에 존재하는 장애물 판단 flag
        self.ranges = [100.0] * 180 #라이다 값 전방 180개 받는 list
        self.ranges_yet = [0.0] * 180 #한 call back 이전 라이다 값 저장 list 
        self.last_time = 0 # lidar call back 함수의 주기 계산 위한 시간 업데이트 변수 1
        self.speed = 0 # 차량의 속도 제어 변수
        self.last_cb_time = None # lidar call back 함수의 주기 게산 위한 시간 업데이트 변수 2
        self.period = 0 #lidar call back 함수의 주기 담는 변수
        self.start_avoid_flag = 0 # 차선변경 시작 flag
        self.go_back_flag = 0 # 차량이 추월하고자 하는 차량의 어깨에 걸렸을 때 lane fallow 상태로 들어가는 flag
        self.count = 0 #속도 계산 시 횟수 카운트


    '''
    cluster_and_average
    같은 러버로 예상되는 라이다 값을 뭉쳐 하나의 좌표로 반환.
    하나의 러버에 하나의 좌표를 추출하기 위함.

    '''
    @staticmethod
    def cluster_and_average(x, y, dist_thresh=0.4): 
        if len(x) == 0:
            return np.array([]), np.array([])
        points = np.stack((x, y), axis=1) #x,y 각각의 1차원 배열을 묶어 2차원 배열로 만듦. [[x1,y1],[x2,y2]...]
        clusters = []
        current = [points[0]] #맨 처음 포인트 지정.
        for p in points[1:]:
            if np.linalg.norm(p - current[-1]) < dist_thresh: #current[-1]: 리스트의 마지막 점과 현재 점 사이의 거리가 기준점 이하라면 
                current.append(p) # 현재 클러스터에 그 점을 추가한다.
            else:
                clusters.append(np.mean(current, axis=0)) # 기준점 이상이라면 새로운 클러스터를 만들어야 함. 현재 클러스터의 평균점을 클러스터라고 지정하고, 다음으로 넘어감..
                current = [p] # 새로운 클러스터 생성.
        clusters.append(np.mean(current, axis=0)) # 마지막 클러스터도 뭉쳐준다.
        cl = np.array(clusters) #np형태로 변환하여 배출
        return cl[:,0], cl[:,1]

    @staticmethod
    def sector_mask(ang, sector): # 장애물의 각도가 설정해준 각도 범위에 포함되는지 검사하는 메서드. 350~20 이렇게 넘어가는 경우도 가능하다.
        lo, hi = np.deg2rad(sector[0]), np.deg2rad(sector[1])
        if lo < hi:
            return (ang >= lo) & (ang <= hi)
        else:
            return (ang >= lo) | (ang <= hi)


    '''
    cb_scan
    라이다 콜백 함수. 라이다 토픽이 들어오면 이 함수가 호출되고, 
    이후 자신 기준 좌 우측의 가장 가까운 러버의 중앙을 추종하며 주행한다.
    이때, 두 러버 모두 비슷한 위치라 안정된 상태라고 판단하면 다음 좌표의 러버들을 보고 주행한다.
    '''
    def cb_scan(self, msg: LaserScan):

        #상대속도 도출 위한 주기 측정#
        now = time.time()

        if self.last_cb_time is None:
            self.last_cb_time = now
        else:
            self.period = now - self.last_cb_time
            self.last_cb_time = now
        
        ############################################
        global ranges, last_time, front_car_vel, front_car_dis
        ang = np.arange(len(msg.ranges))*msg.angle_increment + msg.angle_min
        r   = np.array(msg.ranges)
        m   = (r>0.1) & (r<5.0) #거리 필터 마스킹.. r이 범위 안에있으면 True, 아니라면 False 로 이루어진 배열
        r, ang = r[m], ang[m] #거리 필터링 된 값들만 다시 넣음
        y, x    = r*np.cos(ang), r*np.sin(ang) #좌표계 변환. x는 0도부터 359도 순서로 각 장애물들의 x좌표임. 

        mask_r = self.sector_mask(ang, self.right_sector)
        mask_l = self.sector_mask(ang, self.left_sector) #각도 필터링.. 범위 안에있으면 True, 아니라면 False 로 이루어진 배열
        idy    = np.argsort(y) #x를 기준으로 오름차순 정렬했을 때의 인덱스. x = [5,2,9] 이면 idx[1,0,2] 이런 식으로 나오며, x[idx] 이런식으로 쓰면 정렬한 결과를 실제로 얻을 수 있음.

        self.xl, self.yl = x[idy][mask_r[idy]], y[idy][mask_r[idy]] #각도 필터링
        self.xr, self.yr = x[idy][mask_l[idy]], y[idy][mask_l[idy]]
        #x[idx] : x를 정렬된 순서로 만든 배열. 
        #mask_r[idx] : mask_r도 x의 정렬 순서에 맞춰 재정렬.
        #x[idx][mask_r[idx]]: 정렬된 x 중에서 mask_r가 True인 것만 뽑음

        self.xr, self.yr = self.cluster_and_average(self.xr, self.yr) #클러스터링하여 장애물의 평균 점만 뽑아낸 Numpy 배열
        self.xl, self.yl = self.cluster_and_average(self.xl, self.yl)


        # ranges 길이 일치 유지 (90개 또는 180개)
        # if self.right_state == True:
        #     self.ranges = msg.ranges[90:0:-1]  # 자신의 왼편만 봄 (90개, index 0~89 / 왼쪽 90도부터 정면까지)
        #     self.ranges = [round(d,2) for d in self.ranges]

        # elif self.left_state == True:
        #     self.ranges = msg.ranges[359:269:-1]  # 자신의 오른편만 봄 (90개, index 0~89 / 정면부터 오른쪽 90도까지)
        #     self.ranges = [round(d,2) for d in self.ranges]

        # else:
        self.ranges = msg.ranges[90:0:-1] + msg.ranges[359:269:-1]  # 전방위 감지 (180개) / 왼쪽 90도 부터 오른쪽 90도까지 [0~90~180 순이다.. 왼 - 정면 - 오]
        self.ranges = [round(d,2) for d in self.ranges]

        # 전방 차량 감지 시 상대속도 계산 시작
        if not self.lane_change_flag:
            self.lane_change_trigger(self.ranges)

        if self.lane_change_flag:
            avoid.cal_car_vel(self)

        

        # 오른쪽, 왼쪽에 점이 있는지 검사. 두개는 있어야함
        #has_r = len(self.xr)>1 
        #has_l = len(self.xl)>1
        if(self.lane_change_flag == False)and(self.end_flag == False):
            #print("rubber checking.")
            if len(self.yl) and len(self.yr): #둘다 잡았다면 플래그 체크
                self.rubber_check(self.yl[0],self.yr[0])
            
            #elif len(self.yl) or len(self.yr): #하나만 잡힌다면 장애물 회피하듯이
            #    ym = (- (self.xl[0]) / 2) if len(self.yl) else (- (self.xr[0]) / 2 )
            #    xt = self.look_ahead
            #    steer = -np.arctan2(ym, xt)
            #    self.motor.angle = np.degrees(steer)*self.K*0.1
            #    self.motor.speed = 20
            #    self.pub.publish(self.motor)

            else: #둘다 못잡았다면 아직 못보거나 다 탈출한 것

                if self.rubber_flag: # 플래그 켜진 상태에서 못본 것이니 탈출한다는 것.
                    self.end_flag = True

                    print("end")
                self.rubber_flag = False
            
            #print(self.rubber_flag)
            if self.rubber_flag:
                #print("!!!!!!!!!!!!!!!!!!")
                print(f"L:{self.xl[0],self.yl[0]} R:{self.xr[0],self.yr[0]}") # 가장 가까운 점 두개를 잡아 조향한다.

                if len(self.xl) >= 2 and len(self.xr) >= 2 and abs(self.xl[0] + self.xr[0]) < 1:  # 만약 첫 구조물이 안정화된 상태라면
                    ym = (self.xl[1] + self.xr[1]) / 2  # 더 멀리봐서 핸들을 잡는다.
                    xt = self.look_ahead + 0.5  # pure pursuit 계수 조정. 더 멀리 보고 가야 하니까 !!
                    #print(1)
                elif (len(self.xl) >= 1) and (len(self.xr) >= 1):
                    
                    ym = (self.xl[0] + self.xr[0]) / 2
                    xt = self.look_ahead

                else:
                    pass

                steer = -np.arctan2(ym, xt)  # pure pursuit 알고리즘
                self.motor.angle = np.degrees(steer) * self.K
                self.motor.speed = 25
                self.pub.publish(self.motor)
        

    def run(self):
        while not rospy.is_shutdown():
            plt.pause(0.001)
            rospy.sleep(0.01)


    '''
    rubber_check
    러버 주행 모드에 진입하기 위한 메서드. 일정거리 이하라면 러버 주행 플래그를 켠다.
    또한, 높은 속도에서 들어가기 이전에 사전 준비로 속도를 낮추는 작업도 겸한다.
    '''
    def rubber_check(self,yl,yr):
        if yl < 1.5 and yr < 1.5:
            self.rubber_flag = True
        
        if yl < 5 and yr < 5:
            self.rubber_ready_flag = True

    
    '''
    차량 회피 구간 진입 시 발동.
    전방에 일정거리 이하로 차량이 들어오면 라인 변경 플래그 ON.
    차와의 거리에 따라 자신의 차선이 왼쪽인지 오른쪽인지 판단 후, 플래그를 ON 하는 메서드.
    '''
    def lane_change_trigger(self, lidar_data): 
        # 기본적으로 rubber 중이면 무조건 트리거 꺼놓기
        if self.rubber_flag:
            return False

        # lidar_data 확인
        lidar_data = self.ranges
        

        if not self.lane_change_flag:
            # 전방 거리 평균 사용 (단일 튀는 값보다는 평균이 안전함)
            #print("checking start.")
            front_left_sum = sum(lidar_data[i] for i in range(87, 91))
            front_right_sum = sum(lidar_data[j] for j in range(91, 95))
            total_front_sum = (front_left_sum + front_right_sum)

            if total_front_sum < 200 and not self.lane_change_flag:


                #오른쪽 스타트일때는 잘 되나, 왼쪽 스타트일때 차가 다 돌지 못한 상태에서 들어가버려 제대로 파악 안됨..
                #그냥 전방에 장애물 일정거리 이상 가까워졌다고 판단되면 라인 체인지 플래그만 ON 하고 탈출하자..
                self.lane_change_flag = True
                front_left_sum = 0
                front_right_sum = 0
                total_front_sum = 0

                
                return self.lane_change_flag
            
            




if __name__ == '__main__':
    rospy.loginfo("curve_navigator 시작")
    CurveNavigator().run()
    