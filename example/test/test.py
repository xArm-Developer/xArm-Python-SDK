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
import functools
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI
from xarm.core.utils.log import logger

# logger.setLevel(logger.DEBUG)


xarm = XArmAPI(port='192.168.1.113',
               enable_heartbeat=True,
               enable_report=True,
               report_type='normal')
xarm.motion_enable(True)
xarm.set_state(0)

xarm.set_position(x=300, y=0, z=150, roll=-180, yaw=0, pitch=0, is_radian=False, speed=100, auto_enable=True)
xarm.set_position(x=400, y=100, z=150, roll=-180, yaw=0, pitch=0, is_radian=False, speed=100)
xarm.set_position(x=250, y=0, z=200, roll=-180, yaw=0, pitch=0, is_radian=False, speed=100)

xarm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], speed=50, is_radian=False, wait=True)
xarm.set_servo_angle(angle=[45, 0, 0, 0, 0, 0, 0], speed=50, is_radian=False, wait=True)
xarm.set_servo_angle(angle=[0, -45, 0, 0, 0, 0, 0], speed=50, is_radian=False, wait=True)
xarm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], speed=50, is_radian=False, wait=True)
