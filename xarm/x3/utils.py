# !/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
import functools
from ..core.utils.log import logger
from .code import APIState


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
                if not args[0]._check_is_ready or args[0].ready:
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
            # try:
            #     if args[0].connected:
            #         return func(*args, **kwargs)
            #     else:
            #         logger.error('xArm is not connect')
            #         return APIState.NOT_CONNECTED if _type == 'set' else APIState.NOT_CONNECTED, 'xArm is not connect'
            # except Exception as e:
            #     logger.error('{} - {} - {}'.format(type(e).__name__, func.__name__, e))
            #     return APIState.API_EXCEPTION if _type == 'set' else APIState.API_EXCEPTION, str(e)
        return decorator
    return _xarm_is_pause


# def xarm_is_connected(_type='set'):
#     def _xarm_is_connected(func):
#         @functools.wraps(func)
#         def decorator(*args, **kwargs):
#             try:
#                 if args[0].connected:
#                     return func(*args, **kwargs)
#                 else:
#                     logger.error('xArm is not connect')
#                     return APIState.NOT_CONNECTED if _type == 'set' else APIState.NOT_CONNECTED, 'xArm is not connect'
#             except Exception as e:
#                 logger.error('{} - {} - {}'.format(type(e).__name__, func.__name__, e))
#                 return APIState.API_EXCEPTION if _type == 'set' else APIState.API_EXCEPTION, str(e)
#         return decorator
#     return _xarm_is_connected
#
#
# def xarm_is_ready(_type='set'):
#     def _xarm_is_ready(func):
#         @functools.wraps(func)
#         def decorator(*args, **kwargs):
#             try:
#                 if args[0].connected and kwargs.get('auto_enable', False):
#                     args[0].motion_enable(enable=True)
#                     args[0].set_state(0)
#                 if args[0].connected and args[0].ready:
#                     return func(*args, **kwargs)
#                 elif not args[0].connected:
#                     logger.error('xArm is not connect')
#                     return APIState.NOT_CONNECTED if _type == 'set' else APIState.NOT_CONNECTED, 'xArm is not connect'
#                 else:
#                     logger.error('xArm is not ready')
#                     return APIState.NOT_READY if _type == 'set' else APIState.NOT_READY, 'xArm is not ready'
#             except Exception as e:
#                 logger.error('{} - {} - {}'.format(type(e).__name__, func.__name__, e))
#                 return APIState.API_EXCEPTION if _type == 'set' else APIState.API_EXCEPTION, str(e)
#         return decorator
#     return _xarm_is_ready

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
