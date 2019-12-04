#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Get the property of XArmAPI
    Note: is_radian is different when instantiating, some attribute values are different
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

time.sleep(1)

print('=' * 50)
print('default_is_radian:', arm.default_is_radian)
print('version:', arm.version)
print('state:', arm.state)
print('mode:', arm.mode)
print('cmdnum:', arm.cmd_num)
print('error_code:', arm.error_code)
print('warn_code:', arm.warn_code)
print('collision_sensitivity:', arm.collision_sensitivity)
print('teach_sensitivity:', arm.teach_sensitivity)
print('world_offset:', arm.world_offset)
print('gravity_direction:', arm.gravity_direction)

print('============TCP============')
print('* position:', arm.position)
print('* tcp_jerk:', arm.tcp_jerk)
print('* tcp_load:', arm.tcp_load)
print('* tcp_offset:', arm.tcp_offset)
print('* tcp_speed_limit:', arm.tcp_speed_limit)
print('* tcp_acc_limit:', arm.tcp_acc_limit)

print('===========JOINT===========')
print('* angles:', arm.angles)
print('* joint_jerk:', arm.joint_jerk)
print('* joint_speed_limit:', arm.joint_speed_limit)
print('* joint_acc_limit:', arm.joint_acc_limit)
print('* joints_torque:', arm.joints_torque)

arm.disconnect()



