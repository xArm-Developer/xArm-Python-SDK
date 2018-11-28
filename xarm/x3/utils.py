# !/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import functools
from ..core.utils.log import logger
from .code import APIState


def xarm_is_connected(_type='set'):
    def _xarm_is_connected(func):
        @functools.wraps(func)
        def decorator(*args, **kwargs):
            try:
                if args[0].connected:
                    return func(*args, **kwargs)
                else:
                    logger.error('xArm is not connect')
                    return APIState.NOT_CONNECTED if _type == 'set' else APIState.NOT_CONNECTED, 'xArm is not connect'
            except Exception as e:
                logger.error('{} - {} - {}'.format(type(e).__name__, func.__name__, e))
                return APIState.API_EXCEPTION if _type == 'set' else APIState.API_EXCEPTION, str(e)
        return decorator
    return _xarm_is_connected


def xarm_is_ready(_type='set'):
    def _xarm_is_ready(func):
        @functools.wraps(func)
        def decorator(*args, **kwargs):
            try:
                if args[0].connected and args[0].ready:
                    return func(*args, **kwargs)
                elif not args[0].connected:
                    logger.error('xArm is not connect')
                    return APIState.NOT_CONNECTED if _type == 'set' else APIState.NOT_CONNECTED, 'xArm is not connect'
                else:
                    logger.error('xArm is not ready')
                    return APIState.NOT_READY if _type == 'set' else APIState.NOT_READY, 'xArm is not ready'
            except Exception as e:
                logger.error('{} - {} - {}'.format(type(e).__name__, func.__name__, e))
                return APIState.API_EXCEPTION if _type == 'set' else APIState.API_EXCEPTION, str(e)
        return decorator
    return _xarm_is_ready


