#!/usr/bin/env python3
#car width : 29cm
import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan

distance = None
motor_msg = xycar_motor()
right_box_flag = False
left_box_flag = False
mission_end_flag = False
avoid_state = 0
angle_val = 0
LEFT = 318
RIGHT = 42

'''
avoid_state = 0 -> identify box location (R or L)
avoid_state = 1 -> 1st stage of avoid (turn angle)
avoid_state = 2 -> 2nd stage of avoid (restore angle until 0)
'''

def lidar_callback(data):
    global distance
    global tunnel_state
    distance = data.ranges

def drive(angle, speed):
    global motor_msg
    
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)


def calculate_drive_val():
    
    global motor_msg
    global right_box_flag, left_box_flag
    global avoid_state
    global angle_val
    global mission_end_flag

    speed_val = 5
    angle_val = 0
    right_box_flag_cnt = 0
    left_box_flag_cnt = 0
    ok = 0

    if left_box_flag == False and right_box_flag == False:
        for degree in range(LEFT,360):
            if distance[degree] > 0.05 and distance[degree] < 0.5:

def avoid():

##########################################################################
rospy.init_node('obstacle_avoid')
rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size = 1)
motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

time.sleep(3) #ready to connect
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    ok = 0
    angle, speed = calculate_drive_val()
    drive(angle, speed)
    rate.sleep()
