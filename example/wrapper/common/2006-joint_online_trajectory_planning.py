#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: joint online trajectory planning mode
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

arm.move_gohome(wait=True)
arm.set_servo_angle(angle=[-50, 0, 0, 0, 0, 0, 0], wait=True)

# set mode: joint online trajectory planning mode
# the running command will be interrupted when the next command is received
arm.set_mode(6)
arm.set_state(0)
time.sleep(1)

speed = 50

for i in range(10):
    # run on mode(6)
    # the running command will be interrupted, and run the new command
    arm.set_servo_angle(angle=[120, 0, 0, 0, 0, 0, 0], speed=speed, wait=False)
    time.sleep(1)
    # the running command will be interrupted, and run the new command
    arm.set_servo_angle(angle=[-120, 0, 0, 0, 0, 0, 0], speed=speed, wait=False)
    time.sleep(1)

# set_mode: position mode
arm.set_mode(0)
arm.set_state(0)
arm.move_gohome(wait=True)
arm.disconnect()
