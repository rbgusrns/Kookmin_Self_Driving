# sinhodeng.py

import cv2
import numpy as np

def detect_traffic_light(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # GREEN 영역 정의 
    LOW_GREEN  = np.array([45, 100, 100])
    HIGH_GREEN = np.array([75, 255, 255])

    mask_green = cv2.inRange(hsv, LOW_GREEN, HIGH_GREEN)

    # 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:  # 임계값 조정 가능
            dets.append(("GREEN", area))
            # optionally, draw box
            # x, y, w, h = cv2.boundingRect(cnt)
            # cv2.rectangle(img, (x, y), (x + w, y + h), (0,255,0), 2)

    return img, dets


def sinho_detect(img):
    output, dets = detect_traffic_light(img)
    if dets and dets[0][0] == "GREEN":
        return True
    return False