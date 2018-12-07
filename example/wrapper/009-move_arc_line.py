#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI

"""
Move Arc line(linear arc motion)
    set_position: 
        1. explicit setting is_radian=False, the param roll/yaw/pitch unit is degree (°)
        2. set the same speed to ensure smooth speed
        3. explicit setting radius=0 to move arc line
        4. explicit setting wait=False to nor wait, cache paths as much as possible to ensure that all paths are consistent and smooth
    set_pause_time:
        1. sleep with command cache
    set_servo_angle:
        1. calibrate the offset caused by repeated Cartesian paths
"""

xarm = XArmAPI('192.168.1.113')
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

xarm.reset(wait=True)

paths = [
    [300, 0, 150, -180, 0, 0],
    [300, 200, 250, -180, 0, 0],
    [500, 200, 150, -180, 0, 0],
    [500, -200, 250, -180, 0, 0],
    [300, -200, 150, -180, 0, 0],
    [300, 0, 250, -180, 0, 0]
]

count = 100
xarm.set_position(*paths[0], is_radian=False, wait=True)
_, angles = xarm.get_servo_angle(is_radian=True)
xarm.set_pause_time(1)
while count > 0:
    xarm.set_servo_angle(angle=angles, is_radian=True, speed=50, wait=False)
    for path in paths:
        xarm.set_position(*path, is_radian=False, radius=0, wait=False, speed=300)
    count -= 1

xarm.disconnect()
