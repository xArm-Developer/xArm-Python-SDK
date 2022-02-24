# !/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import time
import math
import functools
from ..core.utils.log import logger
from .code import APIState
from ..core.config.x_config import XCONF


def check_modbus_baud(baud=2000000, _type='set', default=None, host_id=XCONF.TGPIO_HOST_ID):
    def _check_modbus_baud(func):
        @functools.wraps(func)
        def decorator(self, *args, **kwargs):
            code = self.checkset_modbus_baud(baud, host_id=host_id)
            if code != 0:
                logger.error('check modbus baud is failed, code={}'.format(code))
                return code if _type == 'set' else (code, default if default != -99 else [])
            else:
                return func(self, *args, **kwargs)
        return decorator
    return _check_modbus_baud


def xarm_is_connected(_type='set'):
    def _xarm_is_connected(func):
        @functools.wraps(func)
        def decorator(self, *args, **kwargs):
            if self.connected:
                return func(self, *args, **kwargs)
            else:
                logger.error('xArm is not connected')
                return APIState.NOT_CONNECTED if _type == 'set' else (APIState.NOT_CONNECTED, 'xArm is not connect')
        return decorator
    return _xarm_is_connected


def xarm_is_ready(_type='set'):
    def _xarm_is_ready(func):
        @functools.wraps(func)
        def decorator(self, *args, **kwargs):
            if self.connected and kwargs.get('auto_enable', False):
                if not self.ready:
                    self.motion_enable(enable=True)
                    self.set_mode(0)
                    self.set_state(0)
            if self.connected:
                if self.check_xarm_is_ready:
                    return func(self, *args, **kwargs)
                else:
                    logger.error('xArm is not ready')
                    logger.info('Please check the arm for errors. If so, please clear the error first. '
                                'Then enable the motor, set the mode and set the state')
                    return APIState.NOT_READY if _type == 'set' else (APIState.NOT_READY, 'xArm is not ready')
            else:
                logger.error('xArm is not connected')
                return APIState.NOT_CONNECTED if _type == 'set' else (APIState.NOT_CONNECTED, 'xArm is not connect')
        return decorator
    return _xarm_is_ready


def xarm_wait_until_not_pause(func):
    @functools.wraps(func)
    def decorator(self, *args, **kwargs):
        self.wait_until_not_pause()
        return func(self, *args, **kwargs)
    return decorator


def xarm_wait_until_cmdnum_lt_max(func):
    @functools.wraps(func)
    def decorator(self, *args, **kwargs):
        self.wait_until_cmdnum_lt_max()
        return func(self, *args, **kwargs)
    return decorator


def xarm_is_not_simulation_mode(ret=0):
    def _xarm_is_not_simulation_mode(func):
        @functools.wraps(func)
        def decorator(self, *args, **kwargs):
            if not self.check_is_simulation_robot():
                return func(self, *args, **kwargs)
            else:
                return ret
        return decorator
    return _xarm_is_not_simulation_mode


def api_log(func):
    @functools.wraps(func)
    def decorator(self, *args, **kwargs):
        ret = func(self, *args, **kwargs)
        logger.info('{}, ret={}, args={}, kwargs={}'.format(func.__name__, ret, args[1:], kwargs))
        return ret
    return decorator

