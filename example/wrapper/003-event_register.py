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


def callback_cmdnum_changed(item):
    print('cmdnum changed:', item)


def callback_state_changed(item):
    print('state changed:', item)


def callback_connect_changed(item):
    print('connect changed:', item)


def callback_error_warn_changed(item):
    print('error warn changed:', item)


def callback_maable_mtbrake_changed(item):
    print('maable mtbrake changed:', item)


def callback_report_location(item):
    print('location report:', item)

xarm = XArmAPI('192.168.1.113', do_not_open=True)
xarm.register_cmdnum_changed_callback(callback=callback_cmdnum_changed)
xarm.register_state_changed_callback(callback=callback_state_changed)
xarm.register_connect_changed_callback(callback=callback_connect_changed)
xarm.register_error_warn_changed_callback(callback=callback_error_warn_changed)
xarm.register_maable_mtbrake_changed_callback(callback=callback_maable_mtbrake_changed)
xarm.register_report_location_callback(callback=callback_report_location)

xarm.connect()
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

time.sleep(5)
xarm.disconnect()
