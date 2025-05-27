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

code = arm.set_tgpio_modbus_timeout(20)
print('set_tgpio_modbus_timeout, code={}'.format(code))

code = arm.set_tgpio_modbus_baudrate(115200)
print('set_tgpio_modbus_baudrate, code={}'.format(code))
time.sleep(2)

while arm.connected:
    # yinshi open/close test
    data_frame = [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03]
    code, res_data = arm.getset_tgpio_modbus_data(data_frame)
    print('getset_tgpio_modbus_data, code={}, res_data={}'.format(code, res_data))
    time.sleep(2)

    data_frame = [0x01, 0x06, 0x00, 0x0A, 0x03, 0x60]
    code, res_data = arm.getset_tgpio_modbus_data(data_frame)
    print('getset_tgpio_modbus_data, code={}, res_data={}'.format(code, res_data))
    time.sleep(1)
