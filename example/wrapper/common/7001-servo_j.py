#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Servo j
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

arm.move_gohome(wait=True)

arm.set_mode(1)
arm.set_state(0)
time.sleep(0.1)

while arm.connected and arm.state != 4:
    for i in range(100):
        angles = [i, 0, 0, 0, 0, 0, 0]
        ret = arm.set_servo_angle_j(angles)
        print('set_servo_angle_j, ret={}'.format(ret))
        time.sleep(0.01)
    for i in range(100):
        angles = [100-i, 0, 0, 0, 0, 0, 0]
        ret = arm.set_servo_angle_j(angles)
        print('set_servo_angle_j, ret={}'.format(ret))
        time.sleep(0.01)

arm.disconnect()
