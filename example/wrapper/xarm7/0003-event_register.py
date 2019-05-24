#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Example: Event callback registration and release
    Note: Register different event callbacks as needed
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


def callback_cmdnum_changed(item):
    print('cmdnum changed:', item)


def callback_state_changed(item):
    print('state changed:', item)


def callback_connect_changed(item):
    print('connect changed:', item)


def callback_error_warn_changed(item):
    print('error warn changed:', item)


def callback_mtable_mtbrake_changed(item):
    print('maable mtbrake changed:', item)


def callback_report_location(item):
    print('location report:', item)

arm = XArmAPI(ip, do_not_open=True)
arm.register_cmdnum_changed_callback(callback=callback_cmdnum_changed)
arm.register_state_changed_callback(callback=callback_state_changed)
arm.register_connect_changed_callback(callback=callback_connect_changed)
arm.register_error_warn_changed_callback(callback=callback_error_warn_changed)
arm.register_mtable_mtbrake_changed_callback(callback=callback_mtable_mtbrake_changed)
arm.register_report_location_callback(callback=callback_report_location)

arm.connect()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

time.sleep(5)
arm.disconnect()
