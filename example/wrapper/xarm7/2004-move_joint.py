#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
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


arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

speed = 50
arm.set_servo_angle(servo_id=1, angle=90, speed=speed, is_radian=False, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=2, angle=-60, speed=speed, is_radian=False, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=3, angle=-30, speed=speed, is_radian=False, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=1, angle=0, speed=speed, is_radian=False, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=3, angle=0, speed=speed, is_radian=False, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=2, angle=0, speed=speed, is_radian=False, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))

arm.reset(wait=True)

speed = math.radians(50)
arm.set_servo_angle(servo_id=1, angle=math.radians(90), speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=2, angle=math.radians(-60), speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=3, angle=math.radians(-30), speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=1, angle=0, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=3, angle=0, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))
arm.set_servo_angle(servo_id=2, angle=0, speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=False))


arm.reset(wait=True)
arm.disconnect()
