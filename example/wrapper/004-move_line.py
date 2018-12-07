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

"""
Move line(linear motion),
    set_position: 
        1. explicit setting is_radian=False, the param roll/yaw/pitch unit is degree (°)
        2. explicit setting wait=True to wait for the arm to complete
    get_position:
        1. explicit setting is_radian=False, the returned value(roll/yaw/pitch) unit is degree (°)
"""

xarm = XArmAPI('192.168.1.113')
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

xarm.reset(wait=True)

xarm.set_position(x=300, y=0, z=150, roll=-180, yaw=0, pitch=0, speed=100, is_radian=False, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(x=300, y=200, z=250, roll=-180, yaw=0, pitch=0, speed=200, is_radian=False, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(x=500, y=200, z=150, roll=-180, yaw=0, pitch=0, speed=300, is_radian=False, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(x=500, y=-200, z=250, roll=-180, yaw=0, pitch=0, speed=400, is_radian=False, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(x=300, y=-200, z=150, roll=-180, yaw=0, pitch=0, speed=500, is_radian=False, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(x=300, y=0, z=250, roll=-180, yaw=0, pitch=0, speed=600, is_radian=False, wait=True)
print(xarm.get_position(is_radian=False))

xarm.disconnect()
