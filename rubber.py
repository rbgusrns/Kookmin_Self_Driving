#!/usr/bin/env python3
# coding: utf-8
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import XycarMotor
import matplotlib.pyplot as plt

class CurveNavigator:
    def __init__(self):
        rospy.init_node('curve_navigator', anonymous=True)
        plt.ion()                          # interactive mode on
        self.fig, self.ax = plt.subplots() #전체 그림 객체와 그래프 개체
        self.sub = rospy.Subscriber("/scan", LaserScan, self.cb_scan)
        self.pub = rospy.Publisher("xycar_motor", XycarMotor, queue_size=1)
        self.motor = XycarMotor()

        # 피팅 파라미터
        self.right_sector = (1, 89)    # 45±30
        self.left_sector  = (271, 359)  # 315±30
        self.fit_deg     = 1
        self.look_ahead  = 0.5
        self.K           = 2.05
        self.lane_width  = 0.6
        self.left_line_flag = False
        self.right_line_flag = False
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

    def cb_scan(self, msg: LaserScan):
        
        ang = np.arange(len(msg.ranges))*msg.angle_increment + msg.angle_min
        r   = np.array(msg.ranges)
        m   = (r>0.1) & (r<5.0) #거리 필터 마스킹.. r이 범위 안에있으면 True, 아니라면 False 로 이루어진 배열
        r, ang = r[m], ang[m] #거리 필터링 된 값들만 다시 넣음
        y, x    = r*np.cos(ang), r*np.sin(ang) #좌표계 변환. x는 0도부터 359도 순서로 각 장애물들의 x좌표임. 

        mask_r = self.sector_mask(ang, self.right_sector)
        mask_l = self.sector_mask(ang, self.left_sector) #각도 필터링.. 범위 안에있으면 True, 아니라면 False 로 이루어진 배열
        idy    = np.argsort(y) #x를 기준으로 오름차순 정렬했을 때의 인덱스. x = [5,2,9] 이면 idx[1,0,2] 이런 식으로 나오며, x[idx] 이런식으로 쓰면 정렬한 결과를 실제로 얻을 수 있음.

        xl, yl = x[idy][mask_r[idy]], y[idy][mask_r[idy]] #각도 필터링
        xr, yr = x[idy][mask_l[idy]], y[idy][mask_l[idy]]
        #x[idx] : x를 정렬된 순서로 만든 배열. 
        #mask_r[idx] : mask_r도 x의 정렬 순서에 맞춰 재정렬.
        #x[idx][mask_r[idx]]: 정렬된 x 중에서 mask_r가 True인 것만 뽑음

        xr, yr = self.cluster_and_average(xr, yr) #클러스터링하여 장애물의 평균 점만 뽑아낸 Numpy 배열
        xl, yl = self.cluster_and_average(xl, yl)

        # 오른쪽, 왼쪽에 점이 있는지 검사. 두개는 있어야함
        has_r = len(xr)>1 
        has_l = len(xl)>1
        #print(f"L:{xl[0],yl[0]} R:{xr[0],yr[0]}")
        xt = self.look_ahead

        '''
        if has_r and has_l: #둘다 있으면
            pr = np.polyfit(xr, yr, self.fit_deg)
            pl = np.polyfit(xl, yl, self.fit_deg)
            ym = (np.polyval(pr, xt) + np.polyval(pl, xt)) / 2
               
            elif has_r: #오른쪽만 있으면
                pr = np.polyfit(xr, yr, self.fit_deg) 
                ym = np.polyval(pr, xt) + self.lane_width/2
            elif has_l: #왼쪽만 있으면
                pl = np.polyfit(xl, yl, self.fit_deg)
                ym = np.polyval(pl, xt) - self.lane_width/2
            
        else: #둘다 없으면
            return
        '''
        ym = (xl[0] + xr[0]) / 2
        #print(ym)
        steer = -np.arctan2(ym, xt) #pure pursuit 알고리즘.

        # --- 그래프 업데이트 ---
        '''
        if has_r and has_l:
           # print("1")
        
            self.ax.clear()
            # 오른쪽 점 (빨강), 왼쪽 점 (파랑)
            if len(xr)>1:
                self.ax.scatter(xr, yr, label='right clusters', alpha=0.7)
            if len(xl)>1:
                self.ax.scatter(xl, yl, label='left clusters', alpha=0.7)
            # 목표점
            self.ax.scatter([xt], [ym], marker='x', s=100, label='target point')
            self.ax.set_xlabel('x (m)')
            self.ax.set_ylabel('y (m)')
            self.ax.set_title(f'steer={np.degrees(steer)*self.K:.1f}°')
            self.ax.legend(loc='upper right')
            plt.pause(0.001)
        '''
        # -----------------------
        print(ym, steer)
        self.motor.angle = np.degrees(steer)*self.K
        #self.motor.angle =0
        self.motor.speed = 10
        self.pub.publish(self.motor)


    def run(self):
        while not rospy.is_shutdown():
            plt.pause(0.001)
            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.loginfo("curve_navigator 시작")
    CurveNavigator().run()
