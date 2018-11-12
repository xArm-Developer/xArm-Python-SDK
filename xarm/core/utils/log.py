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
    stream_handler_fmt = logger_fmt
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

logger.setLevel(logging.DEBUG)

logger.VERBOSE = logging.VERBOSE
logger.DEBUG = logging.DEBUG
logger.INFO = logging.INFO
logger.WARN = logging.WARN
logger.WARNING = logging.WARNING
logger.ERROR = logging.ERROR
logger.CRITICAL = logging.CRITICAL

logger.verbose = functools.partial(logger.log, logging.VERBOSE)

