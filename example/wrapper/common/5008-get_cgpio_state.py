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
print('state: {} {}'.format(states[0], states[1]))
print('digit_io: {:x} {:x} {:x} {:x}'.format(*states[2:6]))
print('analog: {:f} {:f} {:f} {:f}'.format(*states[6:10]))
print('input_conf: {}'.format(states[10]))
print('output_conf: {}'.format(states[11]))



