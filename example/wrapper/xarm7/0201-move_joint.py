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
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

"""
Move joint,
    set_servo_angle:
        1. explicit setting is_radian=False, the param angle unit is degree (°)
        2. explicit setting wait=True to wait for the arm to complete
    get_servo_angle:
        1. explicit setting is_radian=False, the returned value unit is degree (°)
"""

arm = XArmAPI('192.168.1.113')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

arm.set_servo_angle(angle=[60, 0, 0, 0, 0, 0, 0], speed=30, is_radian=False, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[60, -45, 0, 0, 0, 0, 0], speed=30, is_radian=False, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[60, -45, 0, -60, 0, 0, 0], speed=30, is_radian=False, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[45, -45, -30, -60, 0, 0, 0], speed=30, is_radian=False, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[-45, -45, 0, 0, 0, 0, 0], speed=30, is_radian=False, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], speed=30, is_radian=False, wait=True)
print(arm.get_servo_angle(is_radian=False))

arm.disconnect()
