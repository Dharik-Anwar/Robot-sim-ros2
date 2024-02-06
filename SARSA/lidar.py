# !/usr/bin/env/python3

import numpy as np
from math import *
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

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
def scanDiscretization(state_space, lidar):
    x1 = 2 # Front Left zone (no obstacle detected)
    x2 = 2 # Front Right zone (no obstacle detected)
    x3 = 3 # Front Left sector (no obstacle detected)
    x4 = 3 # Front Right sector (no obstacle detected)

    x5 = 2 # Back Left zone (no obstacle detected)
    x6 = 2 # Back Right zone (no obstacle detected)
    x7 = 3 # Back Left sector (no obstacle detected)
    x8 = 3 # Back Right sector (no obstacle detected)

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

    # Detection of object in front of the robot
    if ( min(lidar[(ANGLE_MAX - HORIZON_WIDTH // 3):(ANGLE_MAX)]) < 1.0 ) or ( min(lidar[(ANGLE_MIN):(ANGLE_MIN + HORIZON_WIDTH // 3)]) < 1.0 ):
        object_front = True
    else:
        object_front = False

    # Detection of object on the front left side of the robot
    if min(lidar[(ANGLE_MIN):(ANGLE_MIN + 2 * HORIZON_WIDTH // 3)]) < 1.0:
        object_front_left = True
    else:
        object_front_left = False

    # Detection of object on the front right side of the robot
    if min(lidar[(ANGLE_MAX - 2 * HORIZON_WIDTH // 3):(ANGLE_MAX)]) < 1.0:
        object_front_right = True
    else:
        object_front_right = False

    # Detection of object on the far front left side of the robot
    if min(lidar[(ANGLE_MIN + HORIZON_WIDTH // 3):(ANGLE_MIN + HORIZON_WIDTH)]) < 1.0:
        object_far_front_left = True
    else:
        object_far_front_left = False

    # Detection of object on the far front right side of the robot
    if min(lidar[(ANGLE_MAX - HORIZON_WIDTH):(ANGLE_MAX - HORIZON_WIDTH // 3)]) < 1.0:
        object_far_front_right = True
    else:
        object_far_front_right = False



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

    # Detection of object in back of the robot
    if ( min(lidar[(ANGLE_MID - HORIZON_WIDTH // 3):(ANGLE_MID)]) < 1.0 ) or ( min(lidar[(ANGLE_MID):(ANGLE_MID + HORIZON_WIDTH // 3)]) < 1.0 ):
        object_back = True
    else:
        object_back = False

    # Detection of object on the back left side of the robot
    if min(lidar[(ANGLE_MID - 2 * HORIZON_WIDTH // 3):(ANGLE_MID)]) < 1.0:
        object_back_left = True
    else:
        object_back_left = False

    # Detection of object on the back right side of the robot
    if min(lidar[(ANGLE_MID):(ANGLE_MID + 2 * HORIZON_WIDTH // 3)]) < 1.0:
        object_back_right = True
    else:
        object_back_right = False

    # Detection of object on the far back left side of the robot
    if min(lidar[(ANGLE_MID - HORIZON_WIDTH):(ANGLE_MID - HORIZON_WIDTH // 3)]) < 1.0:
        object_far_back_left = True
    else:
        object_far_back_left = False

    # Detection of object on the far back right side of the robot
    if min(lidar[(ANGLE_MID + HORIZON_WIDTH // 3):(ANGLE_MID + HORIZON_WIDTH)]) < 1.0:
        object_far_back_right = True
    else:
        object_far_back_right = False



    # The front left sector of the vehicle
    if ( object_front and object_front_left ) and ( not object_far_front_left ):
        x3 = 0 # sector 0
    elif ( object_front_left and object_far_front_left ) and ( not object_front ):
        x3 = 1 # sector 1
    elif object_front and object_front_left and object_far_front_left:
        x3 = 2 # sector 2

    if ( object_front and object_front_right ) and ( not object_far_front_right ):
        x4 = 0 # sector 0
    elif ( object_front_right and object_far_front_right ) and ( not object_front ):
        x4 = 1 # sector 1
    elif object_front and object_front_right and object_far_front_right:
        x4 = 2 # sector 2



    # The back left sector of the vehicle
    if ( object_back and object_back_left ) and ( not object_far_back_left ):
        x7 = 0 # sector 0
    elif ( object_back_left and object_far_back_left ) and ( not object_back):
        x7 = 1 # sector 1
    elif object_back and object_back_left and object_far_back_left:
        x7 = 2 # sector 2

    if ( object_back and object_back_right ) and ( not object_far_back_right ):
        x8 = 0 # sector 0
    elif ( object_back_right and object_far_back_right ) and ( not object_back ):
        x8 = 1 # sector 1
    elif object_back and object_back_right and object_far_back_right:
        x8 = 2 # sector 2

    # Find the state space index of (x1,x2,x3,x4) in Q table
    ss = np.where(np.all(state_space == np.array([x1,x2,x3,x4,x5,x6,x7,x8]), axis = 1))
    state_ind = int(ss[0])

    return ( state_ind, x1, x2, x3 , x4 , x5 , x6, x7 , x8 )


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