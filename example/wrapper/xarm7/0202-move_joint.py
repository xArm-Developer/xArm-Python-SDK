#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Move joint,
    1. explicit setting is_radian=False, set the default unit is degree (°) (not radian)
    set_servo_angle:
        1. explicit setting wait=True to wait for the arm to complete
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from configparser import ConfigParser
parser = ConfigParser()
parser.read('../robot.conf')
try:
    ip = parser.get('xArm', 'ip')
except:
    ip = input('Please input the xArm ip address[192.168.1.194]:')
    if not ip:
        ip = '192.168.1.194'

arm = XArmAPI(ip, is_radian=False)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

arm.set_servo_angle(angle=[60, 0, 0, 0, 0, 0, 0], speed=30, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[60, -60, 0, 0, 0, 0, 0], speed=30, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[45, -60, -10, 0, 0, 0, 0], speed=30, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[-45, -45, 0, 0, 0, 0, 0], speed=30, wait=True)
print(arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], speed=30, wait=True)
print(arm.get_servo_angle(is_radian=False))

arm.disconnect()
