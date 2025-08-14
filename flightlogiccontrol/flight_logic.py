import logging
import pathlib
import sys
import time
import pigpio

import rospy
from skyros.drone import Drone

# Initialize logging
logger = logging.getLogger(name)
logging.basicConfig(level=logging.INFO)

# Initialize pigpio for servo control
pi = pigpio.pi()
SERVO_PIN = 21
pi.set_mode(SERVO_PIN, pigpio.OUTPUT)

# Initialize drone
peer = Drone(pathlib.Path(file).parent / "zenoh-config.json5")
peer.logger.setLevel(logging.INFO)
peer.logger.addHandler(logging.StreamHandler(stream=sys.stdout))
rospy.init_node(peer.name)

with peer:
    logger.info("Drone started...")
    logger.info("Waiting for other drones...")
    peer.wait_for_peer_amount(1)  # Wait for at least one other drone
    logger.info(f"Connected peers: {peer.get_peers()}")

    # Take off
    peer.takeoff(z=1.0, delay=7.5)
    peer.wait(2.5)

    # Navigate with collision avoidance
    waypoints1 = [
        (0.5, 0.03, 1.5),  # A
        (0.5, 0.03, 1.4166666666666667),  # Промежуточная точка A → B
        (0.5, 0.03, 1.3333333333333333),
        (0.5, 0.03, 1.25),  # Промежуточная точка A → B
        (0.5, 0.03, 1.1666666666666667),
        (0.5, 0.03, 1.0833333333333333),
        (0.5, 0.03, 1.0),  # Промежуточная точка A → B
        (0.5, 0.03, 0.9166666666666666),
        (0.5, 0.03, 0.8333333333333334),
        (0.5, 0.03, 0.75),  # Промежуточная точка A → B
        (0.5, 0.03, 0.6666666666666666),
        (0.5, 0.03, 0.5833333333333334),
        (0.5, 0.03, 0.5),  # B
        (0.4166666666666667, 0.03, 0.5),  # Промежуточная точка B → C
        (0.3333333333333333, 0.03, 0.5),
        (0.25, 0.03, 0.5),  # Промежуточная точка B → C
        (0.16666666666666666, 0.03, 0.5),
        (0.08333333333333333, 0.03, 0.5),
        (0.0, 0.03, 0.5),  # Промежуточная точка B → C
        (-0.08333333333333333, 0.03, 0.5),
        (-0.16666666666666666, 0.03, 0.5),
        (-0.25, 0.03, 0.5),  # Промежуточная точка B → C
        (-0.3333333333333333, 0.03, 0.5),
        (-0.4166666666666667, 0.03, 0.5),
        (-0.5, 0.03, 0.5),  # C
    ]

waypoints2 = [
    (-0.5, 0.03, 0.5),  # C
    (-0.5, 0.03, 0.5833333333333334),  # Промежуточная точка C → D
    (-0.5, 0.03, 0.6666666666666666),
    (-0.5, 0.03, 0.75),  # Промежуточная точка C → D
    (-0.5, 0.03, 0.8333333333333334),
    (-0.5, 0.03, 0.9166666666666666),
    (-0.5, 0.03, 1.0),  # Промежуточная точка C → D
    (-0.5, 0.03, 1.0833333333333333),
    (-0.5, 0.03, 1.1666666666666667),
    (-0.5, 0.03, 1.25),  # Промежуточная точка C → D
    (-0.5, 0.03, 1.3333333333333333),
    (-0.5, 0.03, 1.4166666666666667),
    (-0.5, 0.03, 1.5),  # D
    (-0.4166666666666667, 0.03, 1.5),  # Промежуточная точка D → A
    (-0.3333333333333333, 0.03, 1.5),
    (-0.25, 0.03, 1.5),  # Промежуточная точка D → A
    (-0.16666666666666666, 0.03, 1.5),
    (-0.08333333333333333, 0.03, 1.5),
    (0.0, 0.03, 1.5),  # Промежуточная точка D → A
    (0.08333333333333333, 0.03, 1.5),
    (0.16666666666666666, 0.03, 1.5),
    (0.25, 0.03, 1.5),  # Промежуточная точка D → A
    (0.333, 0.03, 1.5),
    (0.4166666666666667, 0.03, 1.5),
    (0.5, 0.03, 1.5),  # A
]

# Select waypoints based on drone identity
waypoints = waypoints1
x_start, y_start, z_start = waypoints[0]

try:
    # Navigate to starting point
    pi.set_servo_pulsewidth(SERVO_PIN, 1000)  # Servo off (not drawing)
    peer.navigate_wait(x=x_start, y=y_start, z=z_start, frame_id="aruco_map")

    for x, y, z in waypoints:
        logger.info(f"Navigating to waypoint: ({x}, {y}, {z})")
        pi.set_servo_pulsewidth(SERVO_PIN, 1000)  # Servo off while moving
        peer.navigate_with_avoidance(x=x, y=y, z=z, frame_id="aruco_map", timeout=20.0)
        peer.wait(1)
        pi.set_servo_pulsewidth(SERVO_PIN, 2000)  # Servo on (drawing)
        time.sleep(0.5)  # Short pause to ensure drawing at waypoint
finally:
    pi.set_servo_pulsewidth(SERVO_PIN, 1000)  # Servo off
    peer.navigate_wait(x=x_start, y=y_start, z=1.0, frame_id="aruco_map")
    peer.land()
    pi.stop()  # Stop pigpio