#!/usr/bin/env python
# -*- coding: utf-8 -*- 
'''
라이다 토픽 받아와서 시각화 해주는 파일.
'''
import rospy
from sensor_msgs.msg import LaserScan #센서 메시지 형태 받아옴.
import matplotlib.pyplot as plt #그래프나 이미지 시각화에 사용하는 클래스. 라이다 거리값 시각화, 카메라 이미지 출력, 궤적 그리기 등...
import numpy as np

fig, ax = plt.subplots(figsize=(8, 8))
'''
그래프 그릴 준비를 하는 기본 코드
plt.subplots() : Figure(그림 전체)와 Axes(그래프 영역 1개)를 생성
figsize : (8,8) 출력되는 그림 사이즈를 8*8로 설정.
fig : 전체 그림 객체
ax : 실제로 그림을 그리는 좌표축 객체. 여기에 .plot()이나, .imshow()등 사용해서 그림 그림.
'''
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
#리미트 설정. -120~120

ax.set_aspect('equal') #x축과 y축의 비율을 1:1로 맞춰줌.


lidar_points, = ax.plot([], [], 'bo')
'''
ax.plot(...) : 그래프에 선이나 점을 그리는 함수
b : 파란색
o : 점모양
ax.plot()은 튜플 형태로 Line2D 객체들을 리턴. 쉼표를 붙여, 첫번째 객체 하나만 꺼내옴..

->lidar_points는 update할 떄 쓰는 핸들 역할!
'''
ranges = None

def callback(data): #콜백함수 선언 메시지 하나만을 인자로 받음
    global ranges #전역변수 내부에서 사용
    ranges = data.ranges[0:360] 
'''
data : LaserScan타입의 메시지.
ranges[0:360] : 0도부터 359도까지의 거리값 리스트. 360개 float.

->0도부터 359도까지의 라이다 거리값을 슬라이싱해서 ranges에 저장.
'''

def main():
    global ranges
    rospy.init_node('lidar_visualizer', anonymous=True) #노드명: lidat_visualizer  | anonymous=True는 중복 방지임. 랜덤 숫자 붙여준다.
    rospy.Subscriber('/scan', LaserScan, callback) 
    """
    rospy.Subscriber : /scan 토픽이 발생할 때마다, 콜백함수 호출. 
    /scan : 구독할 토픽 이름
    LaserScan : 메시지 타입
    callback : 콜백함수. 이 함수는 반드시 메시지를 한개 인자로 받아야 함. 
    """
    plt.ion() # Matplotlib의 인터랙티브 모드 on. GUI 이벤트 루프 멈추지 않도록. 원래 show 이후 멈춰야 하나.. 
    plt.show() #Matplotlib에서 그래프 창을 띄우는 명령어. 그림을 다 그리고, 띄움.
    rate = rospy.Rate(10) #루프를 10Hz로 설정. 초당 10회
    print("Lidar Visualizer Ready ----------")
    
    while not rospy.is_shutdown():
        if ranges is not None:            
            angles = np.linspace(0,2*np.pi, len(ranges))+np.pi/2
            #np.linspace(0, 2π, N) : 0도부터 360도까지 균등하게 N등분한 각도 배열 생성. len(ranges)는 360이므로 1도 간격 배열. ]
            #추가로 전체 각도를 +90도. 기본적으로 LaserScan의 angle 기준은 3시가 0도임. 로봇이 12시를 바라보도록 함.
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            lidar_points.set_data(x, y) #그래프 위에 x,y점들을 실시간으로 업데이트. 360개의 점이 매 주기마다 찍힘.
            fig.canvas.draw_idle() #한가할때 그려라.. 성능 최적화.
            plt.pause(0.01) #그래프를 실시간으로 갱신하기 위해 잠시 멈춰주는 함수. 0.01초 스탑
            '''
            라이다 거리값(ranges)을 2D 평면상의 (x, y) 좌표로 바꿈
            x,y는 원을 그리며 360개가 찍히는데, 중간에 장애물이 있으면 그 윤곽이 찍힘. 없다면 최대 측정 거리가 찍히고..
            '''
        rate.sleep() #주기 맞춰주기

if __name__ == '__main__':
    main()

