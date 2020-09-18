#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Arc Joint
"""

import os
import sys
import time
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

speed = 50

angles = [
    [0, 14, -25, 13, 12.6, 0, 0],
    [-14, 40, -73, 20, 33.4, -13.8, 0],
    [21.9, 50, -80, 30, 36, 29, 0]
]
arm.set_pause_time(1)

for _ in range(10):
    for angle in angles:
        code = arm.set_servo_angle(angle=angle, speed=speed, radius=20, wait=False)

arm.reset(wait=True)
arm.disconnect()
