#!/usr/bin/env python
import rospy
from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import time
import pigpio
import coordinateGeneration
from coordinateGeneration import x_coords, z_coords

#Загрузка координат

## Настройка сервы
# pi = pigpio.pi()
# pi.set_mode(21,pigpio.OUTPUT)
# pi.set_servo_pulsewidth(21, 1000)#1000 - "положение" сервы
# time.sleep(2)
# pi.set_servo_pulsewidth(21, 2000)#2000 - "положение" сервы

rospy.init_node('aruco_navigation')

# ROS-сервисы Clover
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# Изначальный взлет
navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True)
rospy.sleep(5)

# Ждём появления глобальной позиции
def pose_callback(msg):
    global current_pose
    current_pose = msg.pose

rospy.Subscriber('/aruco_map/pose', PoseStamped, pose_callback)
current_pose = None

while current_pose is None and not rospy.is_shutdown():
    rospy.sleep(0.1)

if current_pose:
    rospy.loginfo(f"Global position: x={current_pose.position.x}, y={current_pose.position.y}")

    # Основная полетная логика
    pi = pigpio.pi()
    pi.set_mode(21, pigpio.OUTPUT)

    navigate(x=x_coords[0], y=0, z=z_coords[0], frame_id='aruco_map')
    rospy.sleep(10)
    pi.set_servo_pulsewidth(21, 1000)
    rospy.sleep(2)

    for n in range(len(x_coords)-2):
        navigate(x=x_coords[n+1], y=0, z=z_coords[n+1], frame_id='aruco_map')
        print(x_coords[n+1], z_coords[n+1])
        rospy.sleep(10)

# Посадка
land()