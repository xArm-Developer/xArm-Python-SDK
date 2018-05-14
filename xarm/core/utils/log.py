#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>

import logging
import sys
import os
from logging.handlers import TimedRotatingFileHandler, RotatingFileHandler

log_path = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'log', 'xarm', 'pyufarm')
if not os.path.exists(log_path):
    os.makedirs(log_path)


class Logger(logging.Logger):
    # logger_fmt = '[%(levelname)s] %(asctime)s [%(filename)s:%(funcName)s:%(lineno)d]: %(message)s'
    # logger_fmt = '[%(levelname)s] %(asctime)s [%(pathname)s:%(filename)s:%(lineno)d]: %(message)s'
    logger_fmt = '[%(levelname)s] %(asctime)s [%(pathname)s:%(lineno)d]: %(message)s'
    logger_date_fmt = '%Y-%m-%d %H:%M:%S'
    stream_handler_fmt = logger_fmt
    stream_handler_date_fmt = logger_date_fmt
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.DEBUG)
    stream_handler.setFormatter(logging.Formatter(stream_handler_fmt, stream_handler_date_fmt))

    # rotating_file_handler = RotatingFileHandler(filename=os.path.join(log_path, 'pyufarm.log'), mode='a',
    #                                             maxBytes=10240000, backupCount=30)
    # rotating_file_handler.setLevel(logging.DEBUG)
    # rotating_file_handler.setFormatter(logging.Formatter(stream_handler_fmt, stream_handler_date_fmt))
    #
    # # timed_rotaing_file_handler = TimedRotatingFileHandler(filename=os.path.join(log_path, 'pyufarm.log'), when='D', interval=3, backupCount=10)
    # # timed_rotaing_file_handler.setLevel(logging.DEBUG)
    # # timed_rotaing_file_handler.setFormatter(logging.Formatter(stream_handler_fmt, stream_handler_date_fmt))

    logger = logging.Logger(__name__)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(stream_handler)
    # logger.addHandler(rotating_file_handler)
    # # logger.addHandler(timed_rotaing_file_handler)
    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, 'logger'):
            cls.logger = super(Logger, cls).__new__(cls, *args, **kwargs)
        return cls.logger

logger = Logger(__name__)

