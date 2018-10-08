#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from xarm.wrapper import XArmAPI

xarm = XArmAPI('192.168.1.185')
xarm.motion_enable(True)
xarm.set_state(0)

xarm.reset()
time.sleep(2)
xarm.set_position(x=300, y=0, z=100, roll=-180, yaw=0, pitch=0, is_radian=False, speed=50, wait=True, timeout=20)
xarm.set_position(x=300, y=200, z=100, roll=-180, yaw=0, pitch=0, is_radian=False, speed=50, wait=True, timeout=20)
xarm.set_position(x=300, y=-200, z=100, roll=-180, yaw=0, pitch=0, is_radian=False, speed=50, wait=True, timeout=20)
xarm.set_position(x=250, y=0, z=100, roll=-180, yaw=0, pitch=0, is_radian=False, speed=50, wait=True, timeout=20)

xarm.disconnect()
