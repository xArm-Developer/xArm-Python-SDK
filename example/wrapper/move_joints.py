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
xarm.set_servo_angle(1, 90, is_radian=False, speed=20, wait=True)
xarm.set_servo_angle(2, -45, is_radian=False, speed=20, wait=True)
xarm.set_servo_angle(3, 45, is_radian=False, speed=20, wait=True)
xarm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], is_radian=False, speed=20, wait=True)
xarm.set_servo_angle(angle=[45, -45, 0, 0, 0, 0, 0], is_radian=False, speed=20, wait=True)
xarm.reset()
time.sleep(2)

xarm.disconnect()
