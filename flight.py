#!/usr/bin/env python
import rospy
from clover import srv
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

rospy.init_node('aruco_navigation')

# ROS-сервисы Clover
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# Взлёт на 1.5 метра
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
    # Летим к маркеру 2 (x=1.3, y=0)
    navigate(x=1.3, y=0, z=1.5, frame_id='aruco_map')
    rospy.sleep(10)  # Ждём завершения манёвра

# Посадка
land()