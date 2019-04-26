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


arm = XArmAPI('192.168.1.141')
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

while arm.connected and arm.error_code != 28:
    code, analogs = arm.get_tgpio_analog()
    print('code: {}, analogs={}'.format(code, analogs))
    time.sleep(1)
    code, analog = arm.get_tgpio_analog(1)
    print('code: {}, analog(io1)={}'.format(code, analog))
    time.sleep(0.2)
    code, analog = arm.get_tgpio_analog(2)
    print('code: {}, analog(io2)={}'.format(code, analog))
    time.sleep(0.5)



