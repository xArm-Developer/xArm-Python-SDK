#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Move Arc line(linear arc motion)
    set_position:
        1. explicit setting is_radian=False, the param roll/pitch/yaw unit is degree (°)
        2. set the same speed to ensure smooth speed
        3. explicit setting radius=0 to move arc line
        4. explicit setting wait=False to not wait, cache paths as much as possible to ensure that all paths are consistent and smooth
    set_pause_time:
        1. sleep with command cache
    set_servo_angle:
        1. calibrate the offset caused by repeated Cartesian paths
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from xarm.core.utils.log import pretty_print

arm = XArmAPI('192.168.1.113')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

flag = True


def handle_error(item):
    global flag
    if item['error_code'] != 0:
        flag = False
        arm.emergency_stop()

arm.reset(wait=True)

paths = [
    [300, 0, 150, 180, 0, 0],
    [300, 200, 250, 180, 0, 0],
    [500, 200, 150, 180, 0, 0],
    [500, -200, 250, 180, 0, 0],
    [300, -200, 150, 180, 0, 0],
    [300, 0, 250, 180, 0, 0]
]

arm.set_position(*paths[0], is_radian=False, wait=True)
_, angles = arm.get_servo_angle(is_radian=False)
arm.set_pause_time(0.2)


def move():
    global flag
    ret = arm.set_servo_angle(angle=angles, is_radian=False, speed=50, wait=False)
    if ret < 0:
        pretty_print('set_servo_angle, ret={}'.format(ret), color='red')
        flag = False
        return
    for path in paths:
        if arm.has_error:
            return
        ret = arm.set_position(*path[:6], radius=0, is_radian=False, wait=False, speed=300)
        if ret < 0:
            pretty_print('set_position, ret={}'.format(ret), color='red')
            flag = False
            return

while flag:
    move()

arm.disconnect()
