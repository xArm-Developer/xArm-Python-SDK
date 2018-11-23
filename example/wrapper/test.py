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

logger.setLevel(logger.DEBUG)


xarm = XArmAPI('192.168.1.112', enable_report=True, report_type='normal')

xarm = XArmAPI(port='192.168.1.113',
               enable_heartbeat=True,
               enable_report=True,
               report_type='normal')

time.sleep(2)

print(xarm.get_gripper_position())
print(xarm.set_gripper_position(100, True))
