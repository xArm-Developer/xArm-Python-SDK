#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: task feedback example
    1. requires firmware 2.1.0 and above support
"""

import os
import sys
import time
import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from xarm.core.utils import convert


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


arm = XArmAPI(ip, is_radian=False, max_cmdnum=1000)

arm.set_state(4)
time.sleep(1)
arm.set_mode(0)
arm.set_state(0)
# arm.set_cgpio_digital(0, 0)

def feedback_callback(data):
    cmd_id = convert.bytes_to_u16(data[0:2])
    feedback_type = data[8]
    feedback_funcode = data[9]
    feedback_taskid = convert.bytes_to_u16(data[10:12])
    feedback_code = data[12]
    feedback_us = convert.bytes_to_u64(data[13:21])
    if feedback_type == 1:
        # motion start
        print('[FB] motion task {} starts executing, funcode={}, cmd_id={}, us={}, {}'.format(feedback_taskid, feedback_funcode, cmd_id, feedback_us, datetime.datetime.now()))
    elif feedback_type == 2:
        if feedback_code == 0:
            # motion finish
            print('[FB] motion task {} execution completed, funcode={}, cmd_id={}, us={}, {}'.format(feedback_taskid, feedback_funcode, cmd_id, feedback_us, datetime.datetime.now()))
        elif feedback_code == 2:
            # motion discard
            print('[FB] motion task {} is discarded, funcode={}, cmd_id={}, us={}, {}'.format(feedback_taskid, feedback_funcode, cmd_id, feedback_us, datetime.datetime.now()))        
    elif feedback_type == 4:
        # trigger
        print('[FB] task {} is triggered, funcode={}, cmd_id={}, us={}, {}'.format(feedback_taskid, feedback_funcode, cmd_id, feedback_us, datetime.datetime.now()))
    elif feedback_type == 32:
        # other cmd start
        print('[FB] other cmd {} starts executing, funcode={}, us={}, {}'.format(cmd_id, feedback_funcode, feedback_us, datetime.datetime.now()))
    elif feedback_type == 64:
        if feedback_code == 0:
            # other cmd success
            print('[FB] other cmd {} execution success, funcode={}, us={}, {}'.format(cmd_id, feedback_funcode, feedback_us, datetime.datetime.now()))
        elif feedback_code == 1:
            # other cmd failure
            print('[FB] other cmd {} execution failure, funcode={}, us={}, {}'.format(cmd_id, feedback_funcode, feedback_us, datetime.datetime.now())) 

arm.register_feedback_callback(feedback_callback)
arm.set_feedback_type(14)

radius = 0
code = 0
while arm.connected and arm.state < 4 and code == 0:
    code = arm.set_cgpio_digital(0, 0)
    if code != 0:
        break
    code = arm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, radius=radius, speed=100)
    if code != 0:
        break
    code = arm.set_cgpio_digital_with_xyz(0, 1, xyz=[495, 100, 200], fault_tolerance_radius=2)
    if code != 0:
        break
    code = arm.set_position(x=300, y=100, z=200, roll=180, pitch=0, yaw=0, radius=radius)
    if code != 0:
        break
    code = arm.set_position(x=500, y=100, z=200, roll=180, pitch=0, yaw=0, radius=radius)
    if code != 0:
        break
    code = arm.set_position(x=500, y=-100, z=200, roll=180, pitch=0, yaw=0, radius=radius)
    if code != 0:
        break
    code = arm.set_position(x=300, y=-100, z=200, roll=180, pitch=0, yaw=0, radius=radius)
    if code != 0:
        break
    code = arm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, radius=radius)
    if code != 0:
        break

while arm.connected:
    time.sleep(1)

