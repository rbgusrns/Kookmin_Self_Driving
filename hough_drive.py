#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge

class LaneHDrive:
    # Topics
    CAM_TOPIC = '/usb_cam/image_raw'
    MOTOR_TOPIC = 'xycar_motor'

    #카메라 프레임 크기
    WIDTH = 640
    HEIGHT = 480

    # ROI 설정 (이미지 세로 비율) 
    ROI_Y_START = 0.52
    ROI_Y_END = 0.73
    # 요 구간 사이를 ROI로 지정한다.

    # 차선 기울기 필터 최소값
    MIN_SLOPE = 0.2

    # 제어 상수
    GAIN_VP = 0.2
    GAIN_MP = 0.6
    ANGLE_LIMIT = 45
    SPEED_STRAIGHT = 100
    SPEED_TURN = 50

    # BIRD_EYE_VIEW_FUNCTION CONST
    SOURCE_POINTS = np.float32([[210, 300], [40, 438], [425, 300], [595, 453]])
    #                              L_UP       L_DOWN      R_UP       R_DOWN      
    DESTINATION_POINTS = np.float32([[160, 10], [160, 470], [480, 10], [480, 470]])
    #                                 L_UP        L_DOWN       R_UP      R_DOWN

    # 디버그 모드
    DEBUG = True

    def __init__(self):
        rospy.init_node('h_drive')
        self.bridge = CvBridge()
        self.image = None

        #BEV 변환 행렬 생성
        self.transform_source = cv2.getPerspectiveTransform(self.SOURCE_POINTS, self.DESTINATION_POINTS)

        # 퍼블리셔 및 서브스크라이버 설정
        self.pub_motor = rospy.Publisher(self.MOTOR_TOPIC, XycarMotor, queue_size=1)
        rospy.Subscriber(self.CAM_TOPIC, Image, self.cb_image, queue_size=1)

        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(30)
        rospy.loginfo('LaneHDrive initialized (debug={} )'.format(self.DEBUG))

    def cb_image(self, msg): #이미지 콜백 함수.
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            rospy.logwarn('Failed to convert image')

    def get_roi(self, img):
        h, w = img.shape[:2] #h = 480, w = 640
        y1 = int(h * self.ROI_Y_START)
        y2 = int(h * self.ROI_Y_END)
        return img[y1:y2, :] #세로는 ROI로 자르고, 가로는 전부 사용.
    
    def HSV_Filter(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 노란색 범위
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 흰색 범위
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 마스크 합치기
        mask = cv2.bitwise_or(mask_yellow, mask_white)

        # 마스크 적용
        result = cv2.bitwise_and(img, img, mask=mask)
        #cv2.imshow('mask', mask)
        #cv2.imshow('result', result)

        return result

    def detect_lines(self, gray):
        blur = cv2.GaussianBlur(gray, (5,5), 0) # 가우시안 필터 적용
        #edges = cv2.Canny(blur, 80, 120) # 캐니 엣지 검출
        contours, _ = cv2.findContours(blur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.RETR_EXTERNAL : 가장 바깥쪽 윤곽선만 찾는 옵션.  cv2.CHAIN_APPROX_SIMPLE: 꼭짓점만 저장한다는 옵션. 예를들어 직선이면 양끝점만 저장.
        output = blur.copy()
        cv2.drawContours(output, contours, -1, (0, 255, 0), 2)

        if self.DEBUG:
            #cv2.imshow('Edges', edges)
            cv2.imshow("Contours", output)
            print("-----------------")
            print(contours)
            print("-----------------")
        roi = self.get_roi(blur)
        if self.DEBUG:
            cv2.imshow('ROI Edges', roi)
        lines = cv2.HoughLinesP(roi, 1, np.pi/180, 30, minLineLength=30, maxLineGap=10) #허프 라인 검출
        if self.DEBUG and lines is not None:
            disp = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
            for x1,y1,x2,y2 in lines[:,0]:
                cv2.line(disp, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.imshow('Hough Lines', disp)
        return lines

    def fit_lane(self, lines):
        left_pts, right_pts = [], []
        if lines is None:
            return None, None

        offset = int(self.HEIGHT * self.ROI_Y_START)
        for x1, y1, x2, y2 in lines[:,0]: #허프 변환으로 뽑아낸 모든 라인에 대해 검사
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            if abs(slope) < self.MIN_SLOPE:
                continue
            if slope < 0 and x1 < ( 350 ) and x2 < ( 350 ): # 기울기가 음수이고 왼쪽 화면에 있다면
                left_pts.extend([(x1, y1 + offset), (x2, y2 + offset)]) #roi로 잘라 왔으니 다시 돌려놔야겠지?
            elif slope > 0 and x1 > ( 290 ) and x2 > ( 290 ): # 기울기가 양수이고 오른쪽 화면에 있다면
                right_pts.extend([(x1, y1 + offset), (x2, y2 + offset)])

        def line_params(pts): # 두 점을 받아서 직선의 방정식 계수, 즉 기울기와 절편으로 돌려둠
            if len(pts) < 2:
                return None
            xs, ys = zip(*pts) #직선들을 모조리 받아서 x,y로 쪼갠다.
            m, b = np.polyfit(xs, ys, 1) #점들을 이어 직선을 만듦. 점이 3개 이상이라면 평균으로 근사.
            return m, b 

        lanes = (line_params(left_pts), line_params(right_pts)) #왼쪽과 오른쪽을 대표하는 직선 2개 검출.
        if self.DEBUG:
            display = self.image.copy()
            h, w = display.shape[:2]
            for m_b, color in zip(lanes, [(255,0,0),(0,255,255)]):
                if m_b:
                    m, b = m_b
                    y1 = int(h * self.ROI_Y_START)
                    y2 = int(h * self.ROI_Y_END)
                    x1 = int((y1 - b)/m) if m!=0 else 0
                    x2 = int((y2 - b)/m) if m!=0 else 0
                    #cv2.line(display, (x1,y1), (x2,y2), color, 2)
            #cv2.imshow('Lane Fit', display)
        return lanes

    def compute_control(self, lanes):
        (m_l, b_l), (m_r, b_r) = lanes
        mid_x = self.WIDTH / 2

        if m_l and m_r:
            vp = (b_r - b_l) / (m_l - m_r)
            mp = (((self.HEIGHT * self.ROI_Y_START) - b_l) / m_l + ((self.HEIGHT * self.ROI_Y_START) - b_r) / m_r) / 2
        else:
            vp = mp = mid_x

        vp = np.clip(vp, 0, self.WIDTH)
        angle = self.GAIN_VP * (vp - mid_x) + self.GAIN_MP * (mp - mid_x)
        angle = np.clip(angle, -self.ANGLE_LIMIT, self.ANGLE_LIMIT)

        speed = self.SPEED_STRAIGHT if abs(angle) < 5 else self.SPEED_TURN
        return int(angle), int(speed)

    def run(self):
        while not rospy.is_shutdown():
            if self.image is None:
                self.rate.sleep()
                continue
            hsv_filtered = self.HSV_Filter(self.image)

            bird_eye_view = cv2.warpPerspective(hsv_filtered, self.transform_source, (640, 480)) 
            gray = cv2.cvtColor(bird_eye_view, cv2.COLOR_BGR2GRAY)
            lines = self.detect_lines(gray)
            lanes = self.fit_lane(lines)

            if None not in lanes:
                angle, speed = self.compute_control(lanes)
                self.pub_motor.publish(XycarMotor(angle=angle, speed=speed))

            if self.DEBUG:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            self.rate.sleep()

    def cleanup(self):
        cv2.destroyAllWindows()
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    LaneHDrive().run()
