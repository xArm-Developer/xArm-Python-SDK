#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Direct drive linear motor
Please make sure that the direct drive linear motor is attached to the end.
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from configparser import ConfigParser
parser = ConfigParser()
parser.read('../robot.conf')
try:
    ip = parser.get('xArm', 'ip')
except:
    ip = input('Please input the xArm ip address[192.168.1.196]:')
    if not ip:
        ip = '192.168.1.196'


# arm = XArmAPI(ip)
arm = XArmAPI('192.168.1.196')
arm.motion_enable(True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

code = arm.set_linear_motor_back_origin(wait=True)
print('set linear motor back origin, code={}'.format(code))

# status = 1 is go back on zero successful,
code, status = arm.get_linear_motor_on_zero()
print('get linear motor on zero point, code={}, status={}'.format(code, status))

code = arm.set_linear_motor_enable(True)
print('set linear motor enable, code={}'.format(code))

# set speed 500mm/s
code = arm.set_linear_motor_speed(500)
print('set linear motor speed, code={}'.format(code))

# set position 300mm
code = arm.set_linear_motor_pos(300, wait=True)
print('[wait]set linear motor pos, code={}'.format(code))

# set position 700mm
code = arm.set_linear_motor_pos(700, wait=False)
print('[no wait]set linear motor pos, code={}'.format(code))

