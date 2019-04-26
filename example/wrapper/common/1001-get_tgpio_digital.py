#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Get GPIO Digital
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


arm = XArmAPI('192.168.1.141')
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

last_digitals = [-1, -1]
while arm.connected and arm.error_code != 28:
    code, digitals = arm.get_tgpio_digital()
    if code == 0:
        if digitals[0] == 1 and digitals[0] != last_digitals[0]:
            print('IO1 output high level')
        if digitals[1] == 1 and digitals[1] != last_digitals[1]:
            print('IO2 output high level')
        last_digitals = digitals
    time.sleep(0.1)



