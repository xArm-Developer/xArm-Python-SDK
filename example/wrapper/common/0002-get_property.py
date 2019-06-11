#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Get the property of XArmAPI
    Note: is_radian is different when instantiating, some attribute values are different
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
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



