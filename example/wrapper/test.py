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
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI

xarm = XArmAPI('192.168.1.174', enable_report=True)
# # xarm = XArmAPI('COM5')


time.sleep(3)

# print('gripper_enable:', xarm.gripper_enable(True))
# print('gripper_enable:', xarm.gripper_enable(False))
# print('set_gripper_speed:', xarm.set_gripper_speed(30))
# print('get_gripper_position:', xarm.get_gripper_position())
# # print('set_gripper_position:', xarm.set_gripper_position(20))
# print('get_gripper_err_code:', xarm.get_gripper_err_code())
# print('clean_gripper_error:', xarm.clean_gripper_error())



# xarm.motion_enable()
# xarm.set_state(0)

# xarm.motion_enable(servo_id=1, enable=False)
# xarm.motion_enable(servo_id=1, enable=True)
# xarm.set_servo_detach(servo_id=1)
# time.sleep(20)
# xarm.set_servo_attach(servo_id=1)

# xarm.motion_enable(servo_id=None, enable=True)
# xarm.motion_enable(servo_id=1, enable=True)


# xarm.set_servo_detach(1)

# xarm.set_servo_attach(1)

# xarm.reset()

# print('set_servo_detach', xarm.set_servo_detach())
# print('set_servo_attach', xarm.set_servo_attach())
# print('set_state:', xarm.set_state(0))
# print('get_state:', xarm.get_state())
print('get_version:', xarm.get_version())
# print('get_position:', xarm.get_position())
# print('get_servo_angle:', xarm.get_servo_angle())

# print('get_cmdnum:', xarm.get_cmdnum())
# print('get_err_warn_code:', xarm.get_err_warn_code())
# print('clean_error:', xarm.clean_error())
# print('clean_warn:', xarm.clean_warn())
# print('motion_enable:', xarm.motion_enable())
#
# print('save_conf:', xarm.save_conf())
# print('clean_conf:', xarm.clean_conf())
# print('sleep_instruction:', xarm.sleep_instruction(1))
# print('set_tcp_jerk:', xarm.set_tcp_jerk(1000))
# print('set_tcp_maxacc:', xarm.set_tcp_maxacc(1000))
# print('set_joint_jerk:', xarm.set_joint_jerk(1000))
# print('set_joint_maxacc:', xarm.set_joint_maxacc(1000))
# print('set_tcp_offset:', xarm.set_tcp_offset([0, 0, 0, 0, 0, 0]))
#
# print('get_ik:', xarm.get_ik([172, 0, 132, -3014, 0, 0]))
# print('get_fk:', xarm.get_fk([0, 0, 0, 0, 0, 0, 0]))
# print('is_tcp_limit:', xarm.is_tcp_limit([172, 0, 132, -3014, 0, 0]))
# print('is_joint_limit:', xarm.is_joint_limit([0, 0, 0, 0, 0, 0, 0]))

# pose_list = [[300, 0,    100, -3.14, 0, 0],
#             [300, 100,  100, -3.14, 0, 0],
#             [400, 100,  100, -3.14, 0, 0],
#             [400, -100, 100, -3.14, 0, 0],
#             [300, -100, 100, -3.14, 0, 0],
#             [300, 0,    100, -3.14, 0, 0]]
#
# for _ in range(1):
#     for pos in pose_list:
#         start_time = time.time()
#         ret = xarm.set_position(*pos, is_radian=False)
#         print('move, ret={}, {}'.format(ret, time.time()-start_time))

# angle_list = [
#     [0, 0, 0, 0, 0, 0, 0],
#     [-88, -0.07, 0, 0, 0, 0, 0],
#     [-89.02, 28.21, -0.08, -51.7, 0.1, 23.42, -1.17],
#     [-89.02, 28.18, -0.08, -51.65, 0.1, -66, -1.17],
#     [-89.31, 44.31, -0.01, -82.48, -0.25, -52, -1]
# ]
#
# for _ in range(1):
#     for angle in angle_list:
#         start_time = time.time()
#         ret = xarm.set_servo_angle(angle=angle, is_radian=False)
#         print('move, ret={}, {}'.format(ret, time.time()-start_time))


# print(xarm.set_servo_angle(servo_id=0, angle=90, is_radian=False))
#
# xarm.reset()

# print(xarm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0, 0]))

# while xarm.state != 1:
#     time.sleep(0.001)
# print('start move', time.time())
# while xarm.state == 1:
#     time.sleep(0.001)
# print('stop move', time.time())
# sys.exit(0)


while True:
    time.sleep(1)

