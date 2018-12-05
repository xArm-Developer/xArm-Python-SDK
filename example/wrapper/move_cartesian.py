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
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

xarm.reset(wait=True)
time.sleep(2)

# move line(Linear motion): the roll/yaw/pitch unit is rad
xarm.set_position(x=300, y=0, z=100, roll=-3.14, yaw=0, pitch=0, is_radian=True, speed=50)
# move line(Linear motion): the roll/yaw/pitch unit is °
xarm.set_position(x=300, y=0, z=100, roll=180, yaw=0, pitch=0, is_radian=False, speed=50)
# move line(Linear motion): the roll/yaw/pitch unit is angle, and wait the robot stop or timeout
xarm.set_position(x=400, y=0, z=100, roll=-180, yaw=0, pitch=0, is_radian=False, speed=50, wait=True, timeout=20)

# move arc line(Linear arc motion with interpolation)
xarm.set_sleep_time(0.2)
xarm.set_position(x=400, y=100, z=100, roll=-180, yaw=0, pitch=0, radius=0, is_radian=False, speed=50)
xarm.set_position(x=300, y=100, z=100, roll=-180, yaw=0, pitch=0, radius=0, is_radian=False, speed=50)
xarm.set_position(x=300, y=-100, z=100, roll=-180, yaw=0, pitch=0, radius=0, is_radian=False, speed=50)
xarm.set_position(x=400, y=-100, z=100, roll=-180, yaw=0, pitch=0, radius=0, is_radian=False, speed=50)
xarm.set_position(x=400, y=0, z=100, roll=-180, yaw=0, pitch=0, radius=0, is_radian=False, speed=50)

# move relative
xarm.set_position(y=-100, wait=True, timeout=10)
xarm.set_position(x=100, wait=True, timeout=10)
xarm.set_position(y=100, wait=True, timeout=10)
xarm.set_position(x=-100, wait=True, timeout=10)


xarm.disconnect()
