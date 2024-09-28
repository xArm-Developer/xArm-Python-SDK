#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move line(linear motion), relative move
"""

import os
import sys
import time

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


arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.move_gohome(wait=True)

arm.set_position(x=100, relative=True, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(y=200, z=100, relative=True, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(x=200, z=-100, relative=True, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(y=-400, z=100, relative=True, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(x=-200, z=-100, relative=True, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(y=200, z=100, relative=True, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))

arm.move_gohome(wait=True)
arm.disconnect()
