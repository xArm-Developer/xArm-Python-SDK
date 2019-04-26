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


arm = XArmAPI('192.168.1.145')
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

code, digitals = arm.get_cgpio_digital()
print('get_cgpio_digital, code={}, digitals={}'.format(code, digitals))

code, analogs = arm.get_cgpio_analog()
print('get_cgpio_analog, code={}, analogs={}'.format(code, analogs))


