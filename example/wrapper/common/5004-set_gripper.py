#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Gripper Control
Please make sure that the gripper is attached to the end.
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from configparser import ConfigParser
parser = ConfigParser()
parser.read('../robot.conf')
try:
    ip = parser.get('xArm', 'ip')
except:
    ip = input('Please input the xArm ip address[192.168.1.194]:')
    if not ip:
        ip = '192.168.1.194'


arm = XArmAPI(ip)
arm.motion_enable(True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

code = arm.set_gripper_mode(0)
print('set gripper mode: location mode, code={}'.format(code))

code = arm.set_gripper_enable(True)
print('set gripper enable, code={}'.format(code))

code = arm.set_gripper_speed(5000)
print('set gripper speed, code={}'.format(code))

code = arm.set_gripper_position(600, wait=True)
print('[wait]set gripper pos, code={}'.format(code))

code = arm.set_gripper_position(300, wait=True, speed=8000)
print('[no wait]set gripper pos, code={}'.format(code))

