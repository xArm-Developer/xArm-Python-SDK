#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Connect the robot arm through the serial port
    Note: Need to use a dedicated 485 cable to connect the PC to the control box
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI('COM5')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

time.sleep(5)
arm.disconnect()


