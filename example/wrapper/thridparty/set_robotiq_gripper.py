#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Robotiq Gripper Control
Please make sure that the gripper is attached to the end.
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from xarm.x3.code import APIState
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

ret = arm.robotiq_reset()
print('robotiq_reset, ret={}'.format(ret))

code, ret = arm.robotiq_set_activate()
print('robotiq_set_activate, code={}, ret={}'.format(code, ret))

while arm.connected and (code == 0 or code == APIState.WAIT_FINISH_TIMEOUT):
    code, ret = arm.robotiq_close()
    print('robotiq_close, code={}, ret={}'.format(code, ret))
    code, ret = arm.robotiq_open()
    print('robotiq_open, code={}, ret={}'.format(code, ret))

    # code, ret = arm.robotiq_set_position(0xFF)
    # print('robotiq_set_pos(255), code={}, ret={}'.format(code, ret))
    # code, ret = arm.robotiq_set_position(0)
    # print('robotiq_set_pos(0), code={}, ret={}'.format(code, ret))

if code == APIState.END_EFFECTOR_HAS_FAULT:
    print('robotiq fault code: {}'.format(arm.robotiq_status['gFLT']))

arm.disconnect()


# arm = XArmAPI(ip)
# time.sleep(0.5)
# if arm.warn_code != 0:
#     arm.clean_warn()
# if arm.error_code != 0:
#     arm.clean_error()
#
# ret = arm.core.set_modbus_timeout(5)
# print('set modbus timeout, ret = %d' % (ret[0]))
#
# ret = arm.core.set_modbus_baudrate(115200)
# print('set modbus baudrate, ret = %d' % (ret[0]))
#
# # robotiq open/close test
# data_frame = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
# ret = arm.core.tgpio_set_modbus(data_frame, len(data_frame))
# print('set modbus, ret = %d' % (ret[0]))
# time.sleep(0.1)
#
# data_frame = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
# ret = arm.core.tgpio_set_modbus(data_frame, len(data_frame))
# print('set modbus, ret = %d' % (ret[0]))
#
# arm.disconnect()
