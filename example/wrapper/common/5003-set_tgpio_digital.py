#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Set Tool GPIO Digital
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


arm = XArmAPI(ip, check_robot_sn=False)
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

print('set IO0 high level')
arm.set_tgpio_digital(0, 1)
time.sleep(2)
print('set IO1 high level')
arm.set_tgpio_digital(1, 1)
time.sleep(2)
print('set IO0 low level')
arm.set_tgpio_digital(0, 0)
print('set IO1 low level')
arm.set_tgpio_digital(1, 0)


