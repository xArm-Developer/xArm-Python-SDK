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

arm = XArmAPI(port='192.168.1.145', is_radian=False)

# arm.set_mode(2)
# arm.set_teach_sensitivity(1)
# arm.save_conf()

# print(arm.get_gpio_digital())
# print(arm.get_gpio_analog())
# import time

time.sleep(1)
arm.disconnect()
