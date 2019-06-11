#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Get Tool GPIO Analog
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

while arm.connected and arm.error_code != 19 and arm.error_code != 28:
    code, analogs = arm.get_tgpio_analog()
    print('code: {}, analogs={}'.format(code, analogs))
    time.sleep(1)
    code, analog = arm.get_tgpio_analog(0)
    print('code: {}, analog(io0)={}'.format(code, analog))
    time.sleep(0.2)
    code, analog = arm.get_tgpio_analog(1)
    print('code: {}, analog(io1)={}'.format(code, analog))
    time.sleep(0.5)



