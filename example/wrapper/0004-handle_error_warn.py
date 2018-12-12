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
from xarm.core import ControllerWarn, ControllerError


xarm = XArmAPI(port='192.168.1.113', do_not_open=True)


def hangle_err_warn_changed(item):
    if item['error_code'] != 0:
        controller_error = ControllerError(item['error_code'])
        print('controller error:')
        print('  code: {}, desc: {}'.format(controller_error.code, controller_error.description))
        if 10 <= item['error_code'] <= 17 or item['error_code'] == 28:
            xarm.get_servo_debug_msg(show=True)
            # TODO handle error
        else:
            # TODO handle error
            xarm.clean_error()
            xarm.motion_enable(True)
            xarm.set_state(0)

    else:
        print('error is clean')
    if item['warn_code'] != 0:
        controller_warn = ControllerWarn(item['warn_code'])
        print('controller warn:')
        print('  code: {}, desc: {}'.format(controller_warn.code, controller_warn.description))
        # TODO handle warn
        xarm.clean_warn()
    else:
        print('warnning is clean')


xarm.register_error_warn_changed_callback(hangle_err_warn_changed)
xarm.connect()
xarm.motion_enable(enable=True)
xarm.set_mode(0)
xarm.set_state(state=0)

xarm.reset(wait=True)

time.sleep(3)
xarm.set_position(x=100)

time.sleep(3)

xarm.disconnect()
