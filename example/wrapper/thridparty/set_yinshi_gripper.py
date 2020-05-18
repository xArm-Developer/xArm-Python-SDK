#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: yinshi gripper Control
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
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)

ret = arm.core.set_modbus_timeout(20)
print('set modbus timeout, ret = %d' % (ret[0]))

ret = arm.core.set_modbus_baudrate(115200)
print('set modbus baudrate, ret = %d' % (ret[0]))
time.sleep(2)

while arm.connected:
    # yinshi open/close test
    data_frame = [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03]
    ret = arm.core.tgpio_set_modbus(data_frame, len(data_frame))
    print('set modbus, ret = %d' % (ret[0]))
    time.sleep(2)

    data_frame = [0x01, 0x06, 0x00, 0x0A, 0x03, 0x60]
    ret = arm.core.tgpio_set_modbus(data_frame, len(data_frame))
    print('set modbus, ret = %d' % (ret[0]))
    time.sleep(1)
