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

xarm = XArmAPI(port='192.168.1.113',
               enable_report=True,
               report_type='rich',
               is_radian=False)

# xarm.motion_enable(True)
# xarm.set_mode(0)
# xarm.set_state(0)
print(xarm.version)
# print(xarm.get_position())
print(xarm.device_type)
print(xarm.axis)
# print(xarm.tcp_speed_limit)
# print(xarm.tcp_acc_limit)
# print(xarm.joint_speed_limit)
# print(xarm.joint_acc_limit)


# xarm.motion_enable(True)
# xarm.set_state(0)
# xarm.clean_error()
# xarm.set_state(0)
# time.sleep(5)

# print(xarm.get_servo_angle())
# print(xarm.get_position(is_radian=False))

# print(time.time())
# print(xarm.get_position())
# print(xarm.get_servo_angle())
# print(xarm.is_tcp_limit([300, 0, 100, 180, 0, 0]))
# print(xarm.is_joint_limit([90, -45, 0, 0, 0, 0, 0]))
# print(time.time())

# def test():
#     print('default_is_radian:', xarm.default_is_radian)
#     print('position:', xarm.position, xarm.last_used_position, xarm.position_offset)
#     print('tcp speed/acc:', xarm.last_used_tcp_speed, xarm.last_used_tcp_acc)
#     print('angles:', xarm.angles, xarm.last_used_angles)
#     print('joint speed/acc:', xarm.last_used_joint_speed, xarm.last_used_joint_acc)
#     print('============================================')
# test()
# xarm.set_position(x=300, y=0, z=150, roll=-180, yaw=0, pitch=0, is_radian=False, speed=80, auto_enable=True)
# # test()
# xarm.set_position(x=400, y=100, z=150, roll=-180, yaw=0, pitch=0, is_radian=False, speed=70)
# # test()
# xarm.set_position(x=250, y=0, z=200, roll=-180, yaw=0, pitch=0, is_radian=False, speed=100)
# # test()
#
# xarm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], speed=50, is_radian=False, wait=True)
# test()
# xarm.set_servo_angle(angle=[45, 0, 0, 0, 0, 0, 0], speed=80, is_radian=False, wait=True)
# test()
# xarm.set_servo_angle(angle=[0, -45, 0, 0, 0, 0, 0], speed=40, is_radian=False, wait=True)
# test()
# xarm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0], speed=60, is_radian=False, wait=True)
# test()
