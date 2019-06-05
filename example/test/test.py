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
from xarm.core.config.x_config import XCONF
from xarm.core.utils.log import logger
# logger.setLevel(logger.VERBOSE)

# logger.setLevel(logger.DEBUG)

# arm = XArmAPI(port='192.168.1.194')
# time.sleep(1)
# print(arm.device_type)

# arm.clean_error()
# arm.motion_enable(True)
# arm.set_mode(0)
# arm.set_state(0)

# last_states = [1] * 8
# tcp_speed = 100
#
#
# def handle_error(ret):
#     if ret['error_code'] != 0:
#         arm.clean_error()
#         arm.emergency_stop()
#
# arm.register_error_warn_changed_callback(handle_error)
#
#
# def move_step(io, value):
#     try:
#         # global is_set_gripper, gpos
#         if value == 1:
#             print('stop')
#             arm.emergency_stop()
#             arm._arm._sync()
#             return -1
#         if io == 0:
#             print('x++')
#             arm.set_position(x=tcp_limit[0][1], speed=tcp_speed)
#         elif io == 1:
#             print('x--')
#             arm.set_position(x=tcp_limit[0][0], speed=tcp_speed)
#         elif io == 2:
#             print('y++')
#             arm.set_position(y=tcp_limit[1][1], speed=tcp_speed)
#         elif io == 3:
#             print('y--')
#             arm.set_position(y=tcp_limit[1][0], speed=tcp_speed)
#         elif io == 4:
#             print('z++')
#             arm.set_position(z=tcp_limit[2][1], speed=tcp_speed)
#         elif io == 5:
#             print('z--')
#             arm.set_position(z=tcp_limit[2][0], speed=tcp_speed)
#         elif io == 6:
#             print('move_gohome')
#             arm.move_gohome()
#         elif io == 7:
#             print('emergency_stop')
#             arm.emergency_stop()
#         return 0
#     except Exception as e:
#         print(e)
#         return -1
#
# while True:
#     code, ret = arm.get_cgpio_state()
#     states = [ret[3] >> i & 0x01 for i in range(8)]
#     for i in range(8):
#         if states[i] != last_states[i]:
#             if move_step(i, states[i]) < 0:
#                 break
#     last_states = states
#     time.sleep(0.2)

# code, value = arm.get_cgpio_digital()
# print('get_cgpio_digital, code: {}, value: {}'.format(code, value))
# code, value = arm.get_cgpio_analog()
# print('get_cgpio_analog, code: {}, value: {}'.format(code, value))
# for i in range(8):
#     print(arm.set_cgpio_digital(i, 1))
#     time.sleep(0.5)
# for i in range(8):
#     print(arm.set_cgpio_digital(7-i, 0))
#     time.sleep(0.5)

# print(arm.set_cgpio_digital_output_function(0, 255))
# print(arm.set_cgpio_digital_output_function(3, 255))
# print(arm.set_cgpio_digital_output_function(5, 255))
# print(arm.get_cgpio_state())


arm = XArmAPI(port='192.168.1.194', check_robot_sn=False)

# arm.clean_error()
# arm.motion_enable(True)
# arm.set_mode(0)
# arm.set_state(0)
#
# import random
#
# count = 1
# while arm.error_code == 0:
#     # print(count, arm._arm.arm_cmd.gripper_modbus_set_pos(0))
#     # time.sleep(0.1)
#     pos = random.randint(0, 800)
#     print(count, arm.set_gripper_position(pos, wait=True))
#     # pos = random.randint(0, 500)
#     # count += 1
#     # print(count, arm.set_gripper_position(pos, wait=True))
#     count += 1

# print(arm.sn)
# time.sleep(2)

arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)

# time.sleep(2)
# print(arm.get_version())

arm.move_gohome(wait=True)
angles = [
    [i * 0.1, i * -0.1, 0, 0, 0, 0, 0] for i in range(100)
]
for angle in angles:
    arm.set_servo_angle_j(angle)
    time.sleep(0.01)

time.sleep(0.5)
print(arm.get_servo_angle())

arm.disconnect()
