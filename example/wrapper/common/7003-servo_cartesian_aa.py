#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Set the pose represented by the axis angle pose
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


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)
# go to "Zero Position"
arm.move_gohome()
# go to start position
print(arm.set_position(300, 0, 300, 180, 0, 0, wait=True))
time.sleep(0.5)

# set to mode 1
arm.set_mode(1)
arm.set_state(state=0)
time.sleep(1)
# test move pitch continually, roll+90
rp = 0.2
for i in range(450):
    time.sleep(0.005)
    code = arm.set_servo_cartesian_aa([0, 0, 0, 0, rp, 0], relative=True, wait=False)
    print('set_servo_cartesian_aa, code={}, i={}, rp={}'.format(code, i, rp))



