#!/usr/bin/env python
#car width : 29cm
import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan

ultra_msg = None
distance = None
motor_msg = xycar_motor()
right_box_flag = False
left_box_flag = False
mission_end_flag = False
avoid_state = 0
angle_val = 0
LEFT_OBSTRACT_ANGLE = 318
RIGHT_OBSTRACT_ANGLE = 42
'''
avoid_state = 0 -> identify box location (R or L)
avoid_state = 1 -> 1st stage of avoid (turn angle)
avoid_state = 2 -> 2nd stage of avoid (restore angle until 0)
'''
pub_mission_msg = [0, 0]
sub_mission_msg = [0, 0]

def mission_callback(data):
    global sub_mission_msg
    sub_mission_msg = data.data

def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data 
    
def lidar_callback(data):
    global distance
    global tunnel_state
    distance = data.ranges

def mission_pub(pub_mission_msg):
    pub_msg = Int32MultiArray()
    pub_msg.data = pub_mission_msg
    pub.publish(pub_msg)

def calculate_drive_val():
    global ultra_msg
    global motor_msg
    global right_box_flag, left_box_flag
    global avoid_state
    global angle_val
    global mission_end_flag
    speed_val = 3
    angle_val = 0
    right_box_flag_cnt = 0
    left_box_flag_cnt = 0
    ok = 0
    
    if left_box_flag == False and right_box_flag == False: # 아직 장애물 만나기 이전일 때
      for degree in range(LEFT_OBSTRACT_ANGLE,RIGHT_OBSTRACT_ANGLE): 
          if distance[degree] > 0.05 and distance[degree] < 0.5: #0.5m 이내
            if degree >= 90: #90도가 정면인듯.
               right_box_flag_cnt += 1
            else:
                left_box_flag_cnt += 1
#          print(left_box_flag_cnt, right_box_flag_cnt)
          if right_box_flag_cnt > left_box_flag_cnt and right_box_flag_cnt > 2:
             right_box_flag = True
             avoid_state = 1
             print("right box map")
             mission_pub([7, 0])
          elif right_box_flag_cnt < left_box_flag_cnt and left_box_flag_cnt > 2:
             left_box_flag = True
             avoid_state = 1
             print("left box map")
             mission_pub([7, 0])
    elif right_box_flag == True: #오른쪽에 박스가 있을 때
         if avoid_state == 1: #첫 회피 상태
            angle_val = -50 #왼쪽으로 틀자
            ok = 0
            for degree in range(169,177):
                  if distance[degree] > 0.01 and distance[degree] < 0.45:
                     ok += 1
                  if ok > 3: #확실히 피했으면
                     avoid_state = 2 #다음거 피하기
                     print("1st")
         if avoid_state == 2:
            angle_val = 33 #오른쪽으로 틀어주고
            ok = 0
            for degree in range(2,7):
                  if distance[degree] > 0.01 and distance[degree] < 0.45:
                     ok += 1
                  if ok > 3:
                     avoid_state = 3
                     print("2nd")
         if avoid_state == 3:
            angle_val = -30
            ok = 0
            for degree in range(167,175):
                  if distance[degree] > 0.01 and distance[degree] < 0.45:
                     ok += 1
                  if ok > 3:
                     avoid_state = 4
                     print("3rd")
         if avoid_state == 4:
            angle_val = 30
            if ultra_msg[4] < 50: 
               mission_end_flag = True
               mission_pub([8, 1])               
    elif left_box_flag == True:

         if avoid_state == 1:
            angle_val = 10
            ok = 0
            for degree in range(155,180):
                if distance[degree] < 0.3:
                   ok += 1
                if ok > 5:
                   for degree in range(0,25):
                       if distance[degree] > 0.4:
                          ok += 1
                       if ok > 10:
                          avoid_state = 2
                          print("1st")
         if avoid_state == 2:
            angle_val = -10
            ok = 0
            for degree in range(167,175):
                if distance[degree] < 0.3:
                   ok += 1
                if ok > 5:
                   avoid_state = 3
                   print("2nd")
         if avoid_state == 3:
            angle_val = 10

    return angle_val,speed_val
    	
def drive(angle, speed):
    global motor_msg
    
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

rospy.init_node('obstacle_avoid')
#rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)
rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size = 1)
rospy.Subscriber("mission", Int32MultiArray, mission_callback)
pub = rospy.Publisher("mission", Int32MultiArray, queue_size=1)
motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

time.sleep(3) #ready to connect
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    ok = 0
    if sub_mission_msg[0] == 7:
        angle, speed = calculate_drive_val()
        drive(angle, speed)
   rate.sleep()
