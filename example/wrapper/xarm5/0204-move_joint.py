#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Move joint
    1. explicit setting is_radian=True, set the default unit is radians
    set_servo_angle:
        1. explicit setting wait=True to wait for the arm to complete
"""

import os
import sys
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.113', is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

speed = math.radians(30)

arm.set_servo_angle(angle=[math.radians(60), 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle())
arm.set_servo_angle(angle=[math.radians(60), math.radians(-45), 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle())
arm.set_servo_angle(angle=[math.radians(60), math.radians(-45), 0, math.radians(-60), 0], speed=speed, wait=True)
print(arm.get_servo_angle())
arm.set_servo_angle(angle=[math.radians(45), math.radians(-45), math.radians(-30), math.radians(-60), 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle())
arm.set_servo_angle(angle=[math.radians(-45), math.radians(-45), 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle())
arm.set_servo_angle(angle=[0, 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle())

arm.disconnect()
