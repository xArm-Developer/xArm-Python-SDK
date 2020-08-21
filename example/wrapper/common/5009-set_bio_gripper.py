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

code = arm.set_bio_gripper_enable(True)
print('set_bio_gripper_enable, code={}'.format(code))

code = arm.set_bio_gripper_speed(300)
print('set_bio_gripper_speed, code={}'.format(code))

while arm.connected and arm.error_code == 0:
    code = arm.open_bio_gripper()
    print('open_bio_gripper, code={}'.format(code))
    code = arm.close_bio_gripper()
    print('close_bio_gripper, code={}'.format(code))

