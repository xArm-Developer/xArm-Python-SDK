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


arm = XArmAPI('192.168.1.145')
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

value = 1
for i in range(1, 7):
    code = arm.set_cgpio_digital(i, value)
    print('set_cgpio_digital({}, {}), code={}'.format(i, value, code))

value = 2.6
code = arm.set_cgpio_analog(1, value)
print('set_cgpio_analog(1, {}), code={}'.format(value, code))

value = 3.6
code = arm.set_cgpio_analog(2, value)
print('set_cgpio_analog(2, {}), code={}'.format(value, code))
