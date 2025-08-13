#!/usr/bin/env python
import rospy
from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import time
import math
import pigpio
import json

#Загрузка координат
def load_coordinates(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)
    return [[point[0], point[1]] for point in data]

coordinates = load_coordinates('coordinates.json')
rospy.loginfo(f"Загружено {len(coordinates)} точек")

rospy.init_node('aruco_navigation')

# ROS-сервисы Clover
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def wait_for_position(target_x, target_z, timeout=5.0, tolerance=0.1):
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='aruco_map')

        distance = math.sqrt(
            (telem.x - target_x) ** 2 +
            (telem.z - target_z) ** 2
        )

        if distance < tolerance:
            rospy.loginfo(f"Цель Достигнута, X={telem.x:.2f}, Z={telem.z:.2f}")
            return True
        if rospy.get_time() - start_time > timeout:
            rospy.logwarn(f"Точка не достигнута, текущая позиция: X={telem.x:.2f}, Z={telem.z:.2f}")
            return False
        rospy.sleep(0.2)



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

    # Основная полетная логика для одного дрона
    pi = pigpio.pi()
    pi.set_mode(21, pigpio.OUTPUT)

    for x, z in coordinates:
        rospy.loginfo(f"Летим к точке X={x}, Z={z}")
        set_position(x=x, y=0, z=z, frame_id='aruco_map')

        if wait_for_position(x, z):
            pi.set_servo_pulsewidth(20, 1000)
            rospy.loginfo("Серва активирована")
            rospy.sleep(0.2)
            pi.set_servo_pulsewidth(20, 2000)
            rospy.loginfo("Серва деактивирована")
        else:
            rospy.logerr("Ошибка навигации! Стопаю")
            break

# Посадка
land()
