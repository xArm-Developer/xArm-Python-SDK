# !/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
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
        def decorator(*args, **kwargs):
            code = args[0].checkset_modbus_baud(baud, host_id=host_id)
            if code != 0:
                logger.error('check modbus baud is failed, code={}'.format(code))
                return code if _type == 'set' else (code, default if default != -99 else [])
            else:
                return func(*args, **kwargs)
        return decorator
    return _check_modbus_baud


def xarm_is_connected(_type='set'):
    def _xarm_is_connected(func):
        @functools.wraps(func)
        def decorator(*args, **kwargs):
            if args[0].connected:
                return func(*args, **kwargs)
            else:
                logger.error('xArm is not connect')
                return APIState.NOT_CONNECTED if _type == 'set' else (APIState.NOT_CONNECTED, 'xArm is not connect')
        return decorator
    return _xarm_is_connected


def xarm_is_ready(_type='set'):
    def _xarm_is_ready(func):
        @functools.wraps(func)
        def decorator(*args, **kwargs):
            if args[0].connected and kwargs.get('auto_enable', False):
                if not args[0].ready:
                    args[0].motion_enable(enable=True)
                    args[0].set_mode(0)
                    args[0].set_state(0)
            if args[0].connected:
                if args[0].state_is_ready:
                    return func(*args, **kwargs)
                else:
                    logger.error('xArm is not ready')
                    logger.info('Please check the arm for errors. If so, please clear the error first. '
                                'Then enable the motor, set the mode and set the state')
                    return APIState.NOT_READY if _type == 'set' else (APIState.NOT_READY, 'xArm is not ready')
            else:
                logger.error('xArm is not connect')
                return APIState.NOT_CONNECTED if _type == 'set' else (APIState.NOT_CONNECTED, 'xArm is not connect')
        return decorator
    return _xarm_is_ready


def xarm_is_pause(_type='set'):
    def _xarm_is_pause(func):
        @functools.wraps(func)
        def decorator(*args, **kwargs):
            args[0].check_is_pause()
            return func(*args, **kwargs)
        return decorator
    return _xarm_is_pause


def xarm_wait_until_cmdnum_lt_max(only_wait=False):
    def _xarm_wait_until_cmdnum_lt_max(func):
        @functools.wraps(func)
        def decorator(*args, **kwargs):
            ret = args[0].wait_until_cmdnum_lt_max()
            if not only_wait and ret is not None:
                args[0].log_api_info('API -> {} -> code={}'.format(func.__name__, ret), code=ret)
                return ret
            return func(*args, **kwargs)
        return decorator
    return _xarm_wait_until_cmdnum_lt_max


def xarm_is_not_simulation_mode(ret=0):
    def _xarm_is_not_simulation_mode(func):
        @functools.wraps(func)
        def decorator(*args, **kwargs):
            if not args[0].check_is_simulation_robot():
                return func(*args, **kwargs)
            else:
                return ret
        return decorator
    return _xarm_is_not_simulation_mode


def api_log(func):
    @functools.wraps(func)
    def decorator(*args, **kwargs):
        ret = func(*args, **kwargs)
        logger.info('{}, ret={}, args={}, kwargs={}'.format(func.__name__, ret, args[1:], kwargs))
        return ret
    return decorator


def compare_time(time1, time2):
    try:
        s_time = time.mktime(time.strptime(time1, '%Y-%m-%d'))
        e_time = time.mktime(time.strptime(time2, '%Y-%m-%d'))
        return int(s_time) - int(e_time) > 0
    except:
        return False


def compare_version(v1, v2):
    for i in range(3):
        if v1[i] > v2[i]:
            return True
        elif v1[i] < v2[i]:
            return False
    return False


def filter_invaild_number(num, ndigits=3, default=0.0):
    if math.isnan(num) or math.isinf(num):
        return round(default, 0) if ndigits < 0 else round(default, ndigits)
    return round(num, 0) if ndigits < 0 else round(num, ndigits)
