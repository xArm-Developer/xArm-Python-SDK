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


def to_radian(val, is_radian=False, default=0):
    return default if val is None else float(val) if is_radian else math.radians(val)

