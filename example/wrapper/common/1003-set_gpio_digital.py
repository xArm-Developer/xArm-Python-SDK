#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Set GPIO Digital
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

print('set IO1 high level')
arm.set_gpio_digital(1, 1)
time.sleep(2)
print('set IO2 high level')
arm.set_gpio_digital(2, 1)
time.sleep(2)
print('set IO1 low level')
arm.set_gpio_digital(1, 0)
print('set IO2 low level')
arm.set_gpio_digital(2, 0)


