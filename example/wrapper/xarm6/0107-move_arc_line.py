#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Continuous Move Arc line(linear arc motion)
    continuous_move_arc_line:
        1. explicit setting is_radian=False, the param roll/pitch/yaw unit is degree (°)
        2. explicit setting wait=True to wait for the arm to complete
        3. explicit setting times=10 to movement 10 times
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

paths = [
    [300, 0, 150, -180, 0, 0],
    [300, 200, 250, -180, 0, 0],
    [500, 200, 150, -180, 0, 0],
    [500, -200, 250, -180, 0, 0],
    [300, -200, 150, -180, 0, 0],
    [300, 0, 250, -180, 0, 0],
    [300, 200, 350, -180, 0, 0],
    [500, 200, 250, -180, 0, 0],
    [500, -200, 350, -180, 0, 0],
    [300, -200, 250, -180, 0, 0],
    [300, 0, 350, -180, 0, 0],
]

arm.move_arc_lines(paths, is_radian=False, speed=300, times=0, wait=True)

arm.disconnect()
