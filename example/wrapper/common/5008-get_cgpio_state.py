#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Get Controller GPIO Digital/Anglog
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

code, states = arm.get_cgpio_state()
print('get_cgpio_state, code={}'.format(code))
print('GPIO state: {}'.format(states[0]))
print('GPIO error code: {}'.format(states[1]))
print('Digital->Input->FunctionalIO: {}'.format([states[2] >> i & 0x0001 for i in range(8)]))
print('Digital->Input->ConfiguringIO: {}'.format([states[3] >> i & 0x0001 for i in range(8)]))
print('Digital->Output->FunctionalIO: {}'.format([states[4] >> i & 0x0001 for i in range(8)]))
print('Digital->Output->ConfiguringIO: {}'.format([states[5] >> i & 0x0001 for i in range(8)]))
print('Analog->Input: {}'.format(states[6:8]))
print('Analog->Output: {}'.format(states[8:10]))
print('Digital->Input->Conf: {}'.format(states[10]))
print('Digital->Output->Conf: {}'.format(states[11]))



