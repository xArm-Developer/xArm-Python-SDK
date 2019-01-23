#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Move line(linear motion)
    set_position:
        1. explicit setting is_radian=True, the param roll/yaw/pitch unit is radians
        2. explicit setting wait=True to wait for the arm to complete
    get_position:
        1. explicit setting is_radian=True, the returned value(roll/pitch/yaw) unit is radians
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.113')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

arm.set_position(x=300, y=0, z=150, roll=-3.1415926, pitch=0, yaw=0, speed=100, is_radian=True, wait=True)
print(arm.get_position(is_radian=True))
arm.set_position(x=300, y=200, z=250, roll=-3.1415926, pitch=0, yaw=0, speed=200, is_radian=True, wait=True)
print(arm.get_position(is_radian=True))
arm.set_position(x=500, y=200, z=150, roll=-3.1415926, pitch=0, yaw=0, speed=300, is_radian=True, wait=True)
print(arm.get_position(is_radian=True))
arm.set_position(x=500, y=-200, z=250, roll=-3.1415926, pitch=0, yaw=0, speed=400, is_radian=True, wait=True)
print(arm.get_position(is_radian=True))
arm.set_position(x=300, y=-200, z=150, roll=-3.1415926, pitch=0, yaw=0, speed=500, is_radian=True, wait=True)
print(arm.get_position(is_radian=True))
arm.set_position(x=300, y=0, z=250, roll=-3.1415926, pitch=0, yaw=0, speed=600, is_radian=True, wait=True)
print(arm.get_position(is_radian=True))

arm.disconnect()
