#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Tool Line
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI





arm = XArmAPI('192.168.1.215', baud_checkset=False)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.move_gohome(wait=True)

arm.set_tool_position(x=100, y=0, z=100, roll=0, pitch=0, yaw=0, speed=100, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_tool_position(x=0, y=200, z=0, roll=0, pitch=0, yaw=0, speed=200, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_tool_position(x=200, y=0, z=0, roll=0, pitch=0, yaw=0, speed=300, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_tool_position(x=0, y=-400, z=0, roll=0, pitch=0, yaw=0, speed=400, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_tool_position(x=-200, y=0, z=0, roll=0, pitch=0, yaw=0, speed=500, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))
arm.set_tool_position(x=0, y=200, z=0, roll=0, pitch=0, yaw=0, speed=600, wait=True)
print(arm.get_position(), arm.get_position(is_radian=True))

arm.move_gohome(wait=True)
arm.disconnect()