# !/usr/bin/env/python3

import numpy as np
from math import *
# from std_msgs.msg import String
# from sensor_msgs.msg import LaserScan

MAX_LIDAR_DISTANCE = 5.0
COLLISION_DISTANCE = 0.3
NEARBY_DISTANCE = 1.0

ZONE_0_LENGTH = 0.4
ZONE_1_LENGTH = 0.7

ANGLE_MAX = 360 - 1
ANGLE_MIN = 1 - 1
ANGLE_MID = (ANGLE_MAX + ANGLE_MIN + 1)//2
HORIZON_WIDTH = 45

# Convert LasecScan msg to array
def lidarScan(msgScan):
    distances = np.array([])
    angles = np.array([])

    for i in range(len(msgScan.ranges)):
        angle = degrees(i * msgScan.angle_increment)
        if ( msgScan.ranges[i] > MAX_LIDAR_DISTANCE ):
            distance = MAX_LIDAR_DISTANCE
        elif ( msgScan.ranges[i] <= msgScan.range_min ):
            distance = msgScan.range_min
            # For real robot - protection
            # if msgScan.ranges[i] < 0.01:
            #     distance = MAX_LIDAR_DISTANCE
        else:
            distance = msgScan.ranges[i]

        distances = np.append(distances, distance)
        angles = np.append(angles, angle)

    # distances in [m], angles in [degrees]
    return ( distances, angles )

# Discretization of lidar scan
def scanDiscretization(lidar):
    x1 = 2 # Front Left zone (no obstacle detected)
    x2 = 2 # Front Right zone (no obstacle detected)
    x3 = 2 # Left Back zone (no obstacle detected)
    x4 = 2 # Left Front zone (no obstacle detected)

    x5 = 2 # Back Left zone (no obstacle detected)
    x6 = 2 # Back Right zone (no obstacle detected)
    x7 = 2 # Right Front zone (no obstacle detected)
    x8 = 2 # Right Back zone (no obstacle detected)

    # Find the front left side lidar values of the vehicle
    lidar_front_left = min(lidar[(ANGLE_MIN):(ANGLE_MIN + HORIZON_WIDTH)])
    if ZONE_1_LENGTH > lidar_front_left > ZONE_0_LENGTH:
        x1 = 1 # zone 1
    elif lidar_front_left <= ZONE_0_LENGTH:
        x1 = 0 # zone 0

    # Find the front right side lidar values of the vehicle
    lidar_front_right = min(lidar[(ANGLE_MAX - HORIZON_WIDTH):(ANGLE_MAX)])
    if ZONE_1_LENGTH > lidar_front_right > ZONE_0_LENGTH:
        x2 = 1 # zone 1
    elif lidar_front_right <= ZONE_0_LENGTH:
        x2 = 0 # zone 0

    # Find the back left side lidar values of the vehicle
    lidar_back_left = min(lidar[(ANGLE_MID - HORIZON_WIDTH):(ANGLE_MID)])
    if ZONE_1_LENGTH > lidar_back_left > ZONE_0_LENGTH:
        x5 = 1 # zone 1
    elif lidar_back_left <= ZONE_0_LENGTH:
        x5 = 0 # zone 0

    # Find the back right side lidar values of the vehicle
    lidar_back_right = min(lidar[(ANGLE_MID):(ANGLE_MID + HORIZON_WIDTH)])
    if ZONE_1_LENGTH > lidar_back_right > ZONE_0_LENGTH:
        x6 = 1 # zone 1
    elif lidar_back_right <= ZONE_0_LENGTH:
        x6 = 0 # zone 0

    # Find the left back side lidar values of the vehicle
    lidar_left_back = min(lidar[(ANGLE_MID - 2*HORIZON_WIDTH):(ANGLE_MID - HORIZON_WIDTH)])
    if ZONE_1_LENGTH > lidar_left_back > ZONE_0_LENGTH:
        x3 = 1 # zone 1
    elif lidar_left_back <= ZONE_0_LENGTH:
        x3 = 0 # zone 0

    # Find the left front side lidar values of the vehicle
    lidar_left_front = min(lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN + 2*HORIZON_WIDTH)])
    if ZONE_1_LENGTH > lidar_left_front > ZONE_0_LENGTH:
        x4 = 1 # zone 1
    elif lidar_left_front <= ZONE_0_LENGTH:
        x4 = 0 # zone 0

    # Find the left back side lidar values of the vehicle
    lidar_right_front = min(lidar[(ANGLE_MAX - 2*HORIZON_WIDTH):(ANGLE_MAX - HORIZON_WIDTH)])
    if ZONE_1_LENGTH > lidar_right_front > ZONE_0_LENGTH:
        x7 = 1 # zone 1
    elif lidar_right_front <= ZONE_0_LENGTH:
        x7 = 0 # zone 0

    # Find the left front side lidar values of the vehicle
    lidar_right_back= min(lidar[(ANGLE_MID + HORIZON_WIDTH):(ANGLE_MID + 2*HORIZON_WIDTH)])
    if ZONE_1_LENGTH > lidar_right_back > ZONE_0_LENGTH:
        x8 = 1 # zone 1
    elif lidar_right_back <= ZONE_0_LENGTH:
        x8 = 0 # zone 0

    return [x1, x2, x3 , x4 , x5 , x6, x7 , x8 ]


# Check - crash
def checkCrash(lidar):
    lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1],lidar[(ANGLE_MID-HORIZON_WIDTH):(ANGLE_MID):-1], lidar[(ANGLE_MID):(ANGLE_MID+HORIZON_WIDTH):-1]))
    W = np.linspace(1.2, 1, len(lidar_horizon) // 2)
    W = np.append(W, np.linspace(1, 1.2, len(lidar_horizon) // 2))
    if np.min( W * lidar_horizon ) < COLLISION_DISTANCE:
        return True
    else:
        return False
    
# Check - object nearby
def checkObjectNearby(lidar):
    lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1],lidar[(ANGLE_MID-HORIZON_WIDTH):(ANGLE_MID):-1], lidar[(ANGLE_MID):(ANGLE_MID+HORIZON_WIDTH):-1]))
    W = np.linspace(1.4, 1, len(lidar_horizon) // 2)
    W = np.append(W, np.linspace(1, 1.4, len(lidar_horizon) // 2))
    if np.min( W * lidar_horizon ) < NEARBY_DISTANCE:
        return True
    else:
        return False
    
# Check - goal near
def checkGoalNear(x, y, x_goal, y_goal):
    rho = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    if rho < 0.3:
        return True
    else:
        return False