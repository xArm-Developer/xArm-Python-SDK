#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import time
import functools
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI
from xarm.core.utils.log import logger

# logger.setLevel(logger.DEBUG)

arm = XArmAPI(port='192.168.1.113',
               enable_report=True,
               report_type='rich',
               is_radian=False)

print(arm.get_gpio_digital())
print(arm.get_gpio_analog())

arm.disconnect()