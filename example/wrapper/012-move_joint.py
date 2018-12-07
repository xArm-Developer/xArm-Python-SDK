#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI

"""
Move joint
    set_servo_angle: 
        1. explicit setting is_radian=True, the param roll/yaw/pitch unit is radians
        2. explicit setting wait=True to wait for the arm to complete
    get_servo_angle:
        1. explicit setting is_radian=True, the returned value(roll/yaw/pitch) unit is radians
"""

xarm = XArmAPI('192.168.1.113')
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

xarm.reset(wait=True)

speed = math.radians(30)

xarm.set_servo_angle(angle=[math.radians(60), 0, 0, 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(xarm.get_servo_angle(is_radian=True))
xarm.set_servo_angle(angle=[math.radians(60), math.radians(-45), 0, 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(xarm.get_servo_angle(is_radian=True))
xarm.set_servo_angle(angle=[math.radians(60), math.radians(-45), 0, math.radians(-60), 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(xarm.get_servo_angle(is_radian=True))
xarm.set_servo_angle(angle=[math.radians(45), math.radians(-45), math.radians(-30), math.radians(-60), 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(xarm.get_servo_angle(is_radian=True))
xarm.set_servo_angle(angle=[math.radians(-45), math.radians(-45), 0, 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(xarm.get_servo_angle(is_radian=True))
xarm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(xarm.get_servo_angle(is_radian=True))

xarm.disconnect()
