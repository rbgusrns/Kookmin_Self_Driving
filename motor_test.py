#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy, time
from xycar_msgs.msg import XycarMotor #메시지 형태를 가져옴.

motor = None
motor_msg = XycarMotor() #발행할 메시지 인스턴스

#각도와 속도를 가지고 움직임을 수행.
#각도와 속도 토픽을 받아야 할듯.
#이후 /xycar_motor 토픽으로 발행.
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

rospy.init_node('driver') #node설정. ->driver
motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1) #xycar_motor 토픽 발생. Xycarmotor 메시지 형태이며, 큐사이즈는 1..?
                                                                                        #angle과 speed 포함.          

while not rospy.is_shutdown():
    for i in range(50):
        drive(angle=50.0, speed=10.0)
        time.sleep(0.1)
        
    for i in range(60):
        drive(angle=-50.0, speed=10.0)
        time.sleep(0.1)
