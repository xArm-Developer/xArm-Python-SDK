#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2121, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: cartesian velocity control
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

arm.move_gohome(wait=True)

# set cartesian velocity control mode
arm.set_mode(5)
arm.set_state(0)
time.sleep(1)

arm.vc_set_cartesian_velocity([100, 0, 0, 0, 0, 0])
time.sleep(2)
arm.vc_set_cartesian_velocity([0, -100, 0, 0, 0, 0])
time.sleep(2)
arm.vc_set_cartesian_velocity([0, 0, 100, 0, 0, 0])
time.sleep(2)
arm.vc_set_cartesian_velocity([0, 100, 0, 0, 0, 0])
time.sleep(4)
arm.vc_set_cartesian_velocity([0, 0, -100, 0, 0, 0])
time.sleep(2)
arm.vc_set_cartesian_velocity([0, -100, 0, 0, 0, 0])
time.sleep(2)

# stop
arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])

arm.disconnect()
