#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from xarm.core import ControllerWarn, ControllerError


arm = XArmAPI(port='192.168.1.113', do_not_open=True)


def hangle_err_warn_changed(item):
    if item['error_code'] != 0:
        controller_error = ControllerError(item['error_code'])
        print('controller error:')
        print('  code: {}, desc: {}'.format(controller_error.code, controller_error.description))
        if 10 <= item['error_code'] <= 17 or item['error_code'] == 28:
            arm.get_servo_debug_msg(show=True)
            # TODO handle error
        else:
            # TODO handle error
            arm.clean_error()
            arm.motion_enable(True)
            arm.set_state(0)

    else:
        print('error is clean')
    if item['warn_code'] != 0:
        controller_warn = ControllerWarn(item['warn_code'])
        print('controller warn:')
        print('  code: {}, desc: {}'.format(controller_warn.code, controller_warn.description))
        # TODO handle warn
        arm.clean_warn()
    else:
        print('warnning is clean')


arm.register_error_warn_changed_callback(hangle_err_warn_changed)
arm.connect()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

time.sleep(3)
arm.set_position(x=100)

time.sleep(3)

arm.disconnect()
