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

xarm.set_servo_detach(1)
print('detach')
time.sleep(5)
xarm.set_servo_attach(1)
print('attach')
time.sleep(5)
xarm.disconnect()
