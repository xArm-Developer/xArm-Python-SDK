#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Servo attach/detach
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.113')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# detach a servo
arm.set_servo_detach(servo_id=1)
time.sleep(5)
# attach a servo
arm.set_servo_attach(servo_id=1)
time.sleep(5)
arm.disconnect()
