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

xarm = XArmAPI('192.168.1.185', enable_report=True, report_type='normal')
# xarm.clean_warn()
ret = xarm.set_gripper_enable(True)
print('gripper enable ret:', ret)
ret = xarm.set_gripper_speed(5000)
print('gripper speed ret:', ret)
ret = xarm.get_gripper_position()
print('gripper get pos:', ret)
ret = xarm.set_gripper_position(600)
print('gripper set pos:', ret)
time.sleep(3)
ret = xarm.set_gripper_position(0)
print('gripper set pos:', ret)


while True:
    time.sleep(1)



# while True:
#     xarm = XArmAPI('192.168.1.174', report_type='rich')
#     # time.sleep(0.1)
#     xarm.disconnect()
#     # time.sleep(1)


# xarm = XArmAPI('192.168.1.174', enable_report=True, report_type='normal')

# print(xarm.get_servo_debug_msg())
# xarm = XArmAPI()

# xarm.set_sleep_time(2)


# def test_report_callback(ret, type=None):
#     if type is not None:
#         print('{}: {}, {}'.format(type, ret, time.time()))
#
# xarm.register_report_position_callback(functools.partial(test_report_callback, type='position'))
# xarm.register_report_angles_callback(functools.partial(test_report_callback, type='angles'))

# time.sleep(3)

# print('gripper_enable:', xarm.gripper_enable(True))
# print('gripper_enable:', xarm.gripper_enable(False))
# print('set_gripper_speed:', xarm.set_gripper_speed(30))
# print('get_gripper_position:', xarm.get_gripper_position())
# # print('set_gripper_position:', xarm.set_gripper_position(20))
# print('get_gripper_err_code:', xarm.get_gripper_err_code())
# print('clean_gripper_error:', xarm.clean_gripper_error())


# xarm.motion_enable(servo_id=None, enable=True)
# xarm.set_servo_detach(servo_id=None)

# print(xarm.motion_enable())
# print(xarm.set_state(0))

# xarm.motion_enable(enable=True)
# xarm.motion_enable(servo_id=None, enable=True)
# time.sleep(5)

# xarm.motion_enable(enable=True)
# xarm.set_servo_attach()
# time.sleep(10)
# time.sleep(10)
#
# print('enable:', xarm.motion_enable(enable=True))
# time.sleep(2)
# print('detach:', xarm.set_servo_detach(servo_id=0))
# time.sleep(5)
# print('attach:', xarm.set_servo_attach(servo_id=0))
# # time.sleep(2)
# print('motion enable:', xarm.motion_enable(servo_id=1, enable=True))
# time.sleep(2)

# print('set state:', xarm.set_state(0))
# time.sleep(10)
# print('detach')
# xarm.set_servo_detach(servo_id=1)
# time.sleep(10)
# print('attach')
# xarm.set_servo_attach(servo_id=1)
# print('wait')
# time.sleep(10)
# print('set state')
# xarm.set_state(0)

# xarm.motion_enable(servo_id=None, enable=True)
# xarm.motion_enable(servo_id=1, enable=True)


# xarm.set_servo_detach(1)

# xarm.set_servo_attach(1)

# xarm.reset()

# time.sleep(2)
# print('set_servo_detach', xarm.set_servo_detach(), time.time())
# print('set_servo_attach', xarm.set_servo_attach(), time.time())
# print('set_state:', xarm.set_state(0), time.time())
# print('get_state:', xarm.get_state(), time.time())
# print('get_version:', xarm.get_version(), time.time())
# print('get_position:', xarm.get_position(), time.time())
# print('get_servo_angle:', xarm.get_servo_angle(is_radian=False), time.time())

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

# moving = not (xarm.get_state() != 1 and xarm.get_cmdnum() == 0)
# print('moving:', moving)
#
#
# def xarm_is_moving():
#     global moving
#     if xarm.get_state() == 1:
#         moving = True
#         time.sleep(0.1)
#     else:
#         time.sleep(0.5)
#         if xarm.get_state() != 1 and xarm.get_cmdnum() == 0:
#             if moving:
#                 print('xarm is not moving')
#                 moving = False
#
#
# while True:
#     xarm_is_moving()
#     # time.sleep(1)

