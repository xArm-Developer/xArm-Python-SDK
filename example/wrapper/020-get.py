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

xarm = XArmAPI(port='192.168.1.113')
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

print('=' * 50)
print('version:', xarm.get_version())
print('state:', xarm.get_state())
print('cmdnum:', xarm.get_cmdnum())
print('err_warn_code:', xarm.get_err_warn_code())
print('position(°):', xarm.get_position(is_radian=False))
print('position(radian):', xarm.get_position(is_radian=True))
print('angles(°):', xarm.get_servo_angle(is_radian=False))
print('angles(radian):', xarm.get_servo_angle(is_radian=True))
print('angles(°)(servo_id=1):', xarm.get_servo_angle(servo_id=1, is_radian=False))
print('angles(radian)(servo_id=1):', xarm.get_servo_angle(servo_id=1, is_radian=True))

xarm.disconnect()
