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
        1. explicit setting relative=True to relative move
        2. explicit setting wait=True to wait for the arm to complete
    get_position:
        1. explicit setting is_radian=False, the returned value(roll/yaw/pitch) unit is degree (°)
"""

xarm = XArmAPI('192.168.1.113')
xarm.motion_enable(enable=True)
xarm.set_mode(0)
xarm.set_state(state=0)

xarm.reset(wait=True)

xarm.set_position(x=100, relative=True, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(y=200, z=100, relative=True, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(x=200, z=-100, relative=True, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(y=-400, z=100, relative=True, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(x=-200, z=-100, relative=True, wait=True)
print(xarm.get_position(is_radian=False))
xarm.set_position(y=200, z=100, relative=True, wait=True)
print(xarm.get_position(is_radian=False))

xarm.disconnect()
