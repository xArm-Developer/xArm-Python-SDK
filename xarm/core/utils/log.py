#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import logging
import sys
import os
import functools
from logging.handlers import RotatingFileHandler

log_path = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'log', 'xarm', 'sdk')
if not os.path.exists(log_path):
    os.makedirs(log_path)

logging.VERBOSE = 5
logging.addLevelName(logging.VERBOSE, 'VERBOSE')


class Logger(logging.Logger):
    # logger_fmt = '[%(levelname)s] %(asctime)s [%(pathname)s:%(lineno)d]: %(message)s'
    logger_fmt = '[SDK][%(levelname)s][%(asctime)s][%(filename)s:%(lineno)d]: %(message)s'
    logger_date_fmt = '%Y-%m-%d %H:%M:%S'
    # stream_handler_fmt = logger_fmt
    stream_handler_fmt = '[SDK][%(levelname)s][%(asctime)s][%(lineno)d]: %(message)s'
    stream_handler_date_fmt = logger_date_fmt
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.VERBOSE)
    stream_handler.setFormatter(logging.Formatter(stream_handler_fmt, stream_handler_date_fmt))

    # rotating_file_handler = RotatingFileHandler(filename=os.path.join(log_path, 'xArm-Python-SDK.log'), mode='a',
    #                                             maxBytes=10240000, backupCount=30)
    # rotating_file_handler.setLevel(logging.VERBOSE)
    # rotating_file_handler.setFormatter(logging.Formatter(stream_handler_fmt, stream_handler_date_fmt))

    logger = logging.Logger(__name__)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(stream_handler)
    # logger.addHandler(rotating_file_handler)

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, 'logger'):
            cls.logger = super(Logger, cls).__new__(cls, *args, **kwargs)
        return cls.logger

logger = Logger(__name__)

logger.setLevel(logging.INFO)

logger.VERBOSE = logging.VERBOSE
logger.DEBUG = logging.DEBUG
logger.INFO = logging.INFO
logger.WARN = logging.WARN
logger.WARNING = logging.WARNING
logger.ERROR = logging.ERROR
logger.CRITICAL = logging.CRITICAL

logger.verbose = functools.partial(logger.log, logging.VERBOSE)

colors = {
    'none': '{}',
    'white': '\033[30m{}\033[0m',
    'red': '\033[31m{}\033[0m',
    'green': '\033[32m{}\033[0m',
    'orange': '\033[33m{}\033[0m',
    'blue': '\033[34m{}\033[0m',
    'purple': '\033[35m{}\033[0m',
    'cyan': '\033[36m{}\033[0m',
    'light_gray': '\033[37m{}\033[0m',
    'dark_gray': '\033[90m{}\033[0m',
    'light_red': '\033[91m{}\033[0m',
    'light_green': '\033[92m{}\033[0m',
    'yellow': '\033[93m{}\033[0m',
    'light_blue': '\033[94m{}\033[0m',
    'pink': '\033[95m{}\033[0m',
    'light_cyan': '\033[96m{}\033[0m',
}

level_color_map = {
    logger.VERBOSE: 'cyan',
    logger.DEBUG: 'light_gray',
    logger.INFO: 'blue',
    logger.WARN: 'yellow',
    logger.ERROR: 'light_red',
    logger.CRITICAL: 'red',
}

logger.bak_log = logger._log

if not hasattr(sys, 'frozen'):
    def log(level, msg, args, exc_info=None, extra=None, stack_info=False, color=None):
        if color is None:
            color = level_color_map.get(level, 'none')
        msg = colors.get(color, '{}').format(msg)
        return logger.bak_log(level=level, msg=msg, args=args, exc_info=exc_info, extra=extra, stack_info=stack_info)

    def pretty_print(*args, sep=' ', end='\n', file=None, color='none'):
        msg = ''
        for arg in args:
            msg += arg + sep
        msg = msg.rstrip(sep)
        msg = colors.get(color, '{}').format(msg)
        print(msg, end=end, file=file)
else:
    def log(level, msg, args, exc_info=None, extra=None, stack_info=False, color=None):
        return logger.bak_log(level=level, msg=msg, args=args, exc_info=exc_info, extra=extra, stack_info=stack_info)

    def pretty_print(*args, sep=' ', end='\n', file=None, color='none'):
        msg = ''
        for arg in args:
            msg += arg + sep
        msg = msg.rstrip(sep)
        print(msg, end=end, file=file)

logger._log = log

