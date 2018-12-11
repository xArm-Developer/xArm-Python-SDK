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

xarm = XArmAPI(port='192.168.1.113', is_radian=False)
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

print('=' * 50)
print('default_is_radian:', xarm.default_is_radian)
print('version:', xarm.version)
print('state:', xarm.state)
print('cmdnum:', xarm.cmd_num)
print('err_code:', xarm.error_code)
print('warn_code:', xarm.warn_code)
print('position(°):', xarm.position)
print('angles(°):', xarm.angles)
print('last_used_position:', xarm.last_used_position)
print('last_used_angles:', xarm.last_used_angles)

xarm.disconnect()


xarm = XArmAPI(port='192.168.1.113', is_radian=True)
xarm.motion_enable(enable=True)
xarm.set_mode(0)
xarm.set_state(state=0)

print('*' * 50)
print('default_is_radian:', xarm.default_is_radian)
print('version:', xarm.version)
print('state:', xarm.state)
print('cmdnum:', xarm.cmd_num)
print('err_code:', xarm.error_code)
print('warn_code:', xarm.warn_code)
print('position(°):', xarm.position)
print('angles(°):', xarm.angles)
print('last_used_position:', xarm.last_used_position)
print('last_used_angles:', xarm.last_used_angles)

xarm.disconnect()