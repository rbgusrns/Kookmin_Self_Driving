#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

class LidarViewer:
    def __init__(self, node_name, topic, queue_size, left_deg, right_deg, tol):
        self.left_deg = left_deg -90
        self.right_deg = right_deg -90
        self.tol = tol      # ±tol
        rospy.init_node(node_name, anonymous=False)
        plt.ion()

        rospy.Subscriber(topic, LaserScan, self.callback, queue_size=queue_size)

    def callback(self, data):
        ran = np.array(data.ranges)
        inc = data.angle_increment
        if len(ran) != 360:
            return

        # 0° 기준을 90°가 정면이 되도록 재배치
        ranges = np.zeros(360)
        ranges[270:360] = ran[0:90]
        ranges[0:270] = ran[90:360]

        # 좌표 계산
        degs = np.arange(360)
        rads = degs * inc
        xs = ranges * np.cos(rads)
        ys = -ranges * np.sin(rads)

        # 그래프 세팅
        plt.cla()
        plt.xlim([-5, 5]); plt.ylim([-5, 5])
        plt.scatter(ys, xs, s=2)

        max_r = np.nanmax(ranges[np.isfinite(ranges)])  # 채우기 반경

        for center in (self.left_deg, self.right_deg):
            th1 = np.deg2rad(center - self.tol)
            th2 = np.deg2rad(center + self.tol)
            thetas = np.linspace(th1, th2, 100)
            x_fill = max_r * np.cos(thetas)
            y_fill = -max_r * np.sin(thetas)
            # 부채꼴 채우기
            plt.fill_between(y_fill, x_fill, 0, alpha=0.2)
            # 경계선
            x1, y1 = max_r*np.cos(th1), -max_r*np.sin(th1)
            x2, y2 = max_r*np.cos(th2), -max_r*np.sin(th2)
            plt.plot([0, y1], [0, x1], linewidth=1)
            plt.plot([0, y2], [0, x2], linewidth=1)

        plt.pause(0.001)
        

if __name__ == '__main__':
    # LEFT=60°, RIGHT=120°, 폭=15°
    LidarViewer("viewer", "/scan", 1, left_deg=-45, right_deg=45, tol=45)
    rospy.spin()
