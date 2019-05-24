#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Getting attributes
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
arm.set_state(state=0)

print('=' * 50)
print('default_is_radian:', arm.default_is_radian)
print('version:', arm.version)
print('state:', arm.state)
print('cmdnum:', arm.cmd_num)
print('err_code:', arm.error_code)
print('warn_code:', arm.warn_code)
print('position(°):', arm.position)
print('angles(°):', arm.angles)
print('last_used_position:', arm.last_used_position)
print('last_used_angles:', arm.last_used_angles)

arm.disconnect()


arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

print('*' * 50)
print('default_is_radian:', arm.default_is_radian)
print('version:', arm.version)
print('state:', arm.state)
print('cmdnum:', arm.cmd_num)
print('err_code:', arm.error_code)
print('warn_code:', arm.warn_code)
print('position(°):', arm.position)
print('angles(°):', arm.angles)
print('last_used_position:', arm.last_used_position)
print('last_used_angles:', arm.last_used_angles)

arm.disconnect()
