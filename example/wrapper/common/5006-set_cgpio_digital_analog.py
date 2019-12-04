#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Set Controller GPIO Digital/Analog
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
    ip = input('Please input the xArm ip address[192.168.1.194]:')
    if not ip:
        ip = '192.168.1.194'

arm = XArmAPI(ip)
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

value = 0
for i in range(8):
    code = arm.set_cgpio_digital(i, value)
    print('set_cgpio_digital({}, {}), code={}'.format(i, value, code))
    time.sleep(0.5)

value = 1
for i in range(8):
    code = arm.set_cgpio_digital(i, value)
    print('set_cgpio_digital({}, {}), code={}'.format(i, value, code))
    time.sleep(0.5)

value = 2.6
code = arm.set_cgpio_analog(0, value)
print('set_cgpio_analog(0, {}), code={}'.format(value, code))

value = 3.6
code = arm.set_cgpio_analog(1, value)
print('set_cgpio_analog(1, {}), code={}'.format(value, code))
