#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Set reduced mode
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

x_max, x_min, y_max, y_min, z_max, z_min = 500, -500, 600, -600, 400, -400
code = arm.set_reduced_tcp_boundary([x_max, x_min, y_max, y_min, z_max, z_min])
print('set_reduced_tcp_boundary, code={}'.format(code))
code = arm.set_fense_mode(True)
print('set_fense_mode, code={}'.format(code))
