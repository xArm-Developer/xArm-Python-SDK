#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import logging
import functools
import sys
import os

log_path = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'log', 'xarm', 'sdk')
if not os.path.exists(log_path):
    os.makedirs(log_path)

logging.VERBOSE = 5
logging.addLevelName(logging.VERBOSE, 'VERBOSE')


class Logger(logging.Logger):
    logger_fmt = '{}[%(levelname)s][%(asctime)s][%(filename)s:%(lineno)d] - - %(message)s'
    logger_date_fmt = '%Y-%m-%d %H:%M:%S'
    stream_handler_fmt = logger_fmt.format('[SDK]')
    stream_handler_date_fmt = logger_date_fmt
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.VERBOSE)
    stream_handler.setFormatter(logging.Formatter(stream_handler_fmt, stream_handler_date_fmt))

    logger = logging.Logger(__name__)
    logger.setLevel(logging.VERBOSE)
    logger.addHandler(stream_handler)

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, 'logger'):
            cls.logger = super(Logger, cls).__new__(cls, *args, **kwargs)
        return cls.logger

logger = Logger(__name__)
logger.setLevel(logging.WARNING)

logger.VERBOSE = logging.VERBOSE
logger.DEBUG = logging.DEBUG
logger.INFO = logging.INFO
logger.WARN = logging.WARN
logger.WARNING = logging.WARNING
logger.ERROR = logging.ERROR
logger.CRITICAL = logging.CRITICAL

findCaller = logger.findCaller


def log(msg, *args, **kwargs):
    level = kwargs.pop('level', logger.INFO)
    if logger.findCaller == findCaller:
        rv = findCaller(kwargs.pop('stack_info', False))
        logger.findCaller = lambda x: rv
    logger.log(level, msg, *args, **kwargs)
    if logger.findCaller != findCaller:
        logger.findCaller = findCaller

logger.verbose = functools.partial(logger.log, logger.VERBOSE)

# logger.verbose = functools.partial(log, level=logger.VERBOSE)
# logger.debug = functools.partial(log, level=logger.DEBUG)
# logger.info = functools.partial(log, level=logger.INFO)
# logger.warning = functools.partial(log, level=logger.WARNING)
# logger.error = functools.partial(log, level=logger.ERROR)
# logger.critical = functools.partial(log, level=logger.CRITICAL)

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


def pretty_print(*args, sep=' ', end='\n', file=None, color='none'):
    msg = ''
    for arg in args:
        msg += arg + sep
    msg = msg.rstrip(sep)
    # msg = colors.get(color, '{}').format(msg)
    print(msg, end=end, file=file)
