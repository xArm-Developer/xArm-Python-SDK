#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: yinshi RH56 gripper Control
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

# 各自由度角度、力控、速度寄存器地址
FINGER_ADDR = {
    0: 0x05CE,  # 小拇指
    1: 0x05D0,  # 无名指
    2: 0x05D2,  # 中指
    3: 0x05D4,  #食指
    4: 0x05D6,  #大拇指弯曲
    5: 0x05D8,  #大拇指旋转
}
FORCE_ADDR = {
    0: 0x05DA,  # 小拇指
    1: 0x05DC,
    2: 0x05DE,
    3: 0x05E0,
    4: 0x05E2,
    5: 0x05E4,
}
SPEED_ADDR = {
    0: 0x05F2,  # 小拇指
    1: 0x05F4,
    2: 0x05F6,
    3: 0x05F8,
    4: 0x05FA,
    5: 0x05FC,
}

HAND_ID = 0x01

def set_rh56_finger_speed(arm, finger_id, speed):
    speed = min(max(round(speed), 0), 1000)
    addr = SPEED_ADDR[finger_id]
    modbus = [HAND_ID, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (speed >> 8) & 0xFF, speed & 0xFF]
    code, res_data = arm.getset_tgpio_modbus_data(modbus, timeout=100)
    if code != 0:
        print('[FAILED] set the speed of finger-{}, code={}, res_data={}'.format(finger_id, code, res_data))
    else:
        print('[SUCCESS] set the speed of finger-{}, res_data={}'.format(finger_id, res_data))
    return code, res_data

def set_rh56_finger_force(arm, finger_id, force):
    force = min(max(round(force), 0), 1000)
    addr = FORCE_ADDR[finger_id]
    modbus = [HAND_ID, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (force >> 8) & 0xFF, force & 0xFF]
    code, res_data = arm.getset_tgpio_modbus_data(modbus, timeout=100)
    if code != 0:
        print('[FAILED] set the force of finger-{}, code={}, res_data={}'.format(finger_id, code, res_data))
    else:
        print('[SUCCESS] set the force of finger-{}, res_data={}'.format(finger_id, res_data))
    return code, res_data

def set_rh56_finger_pos(arm, finger_id, pos):
    pos = min(max(round(pos), 0), 1000) if pos != -1 else 0xFFFF
    addr = FINGER_ADDR[finger_id]
    modbus = [HAND_ID, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF]
    code, res_data = arm.getset_tgpio_modbus_data(modbus, timeout=100)
    if code != 0:
        print('[FAILED] set the pos of finger-{}, code={}, res_data={}'.format(finger_id, code, res_data))
    else:
        print('[SUCCESS] set the pos of finger-{}, res_data={}'.format(finger_id, res_data))
    return code, res_data

def set_rh56_finger(arm, pos, speed=1000, force=500, open_threshold=500):
    # 决定执行顺序：闭合用正序，张开用反序。位置大于500认为张开，小于等于500认为是闭合。
    if pos <= open_threshold:
        finger_order = range(0, 5)  # 抓取（闭合）
    else:
        finger_order = reversed(range(0, 5))  # 张开

    for finger_id in finger_order:
        set_rh56_finger_speed(arm, finger_id, speed)
        set_rh56_finger_force(arm, finger_id, force)
        set_rh56_finger_pos(arm, finger_id, pos)
        time.sleep(0.01)

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

for i in range(100):
    set_rh56_finger(arm, pos=1000, force=500,speed=1000)
    time.sleep(2)
    set_rh56_finger(arm, pos=0, force=500, speed=1000)
    time.sleep(2)
