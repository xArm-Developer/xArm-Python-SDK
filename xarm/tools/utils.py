#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
import math
import time
import sys
import traceback


def pprint(*args, **kwargs):
    try:
        stack_tuple = traceback.extract_stack()[0]
        # filename = stack_tuple[0]
        linenumber = stack_tuple[1]
        # funcname = stack_tuple[2]
        print('[{}][line:{}]'.format(
            time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())),
            linenumber
        ), end=' ')
    except:
        pass
    print(*args, **kwargs)


def is_prime(n):
    def _is_prime():
        for i in range(6, int(math.sqrt(n) + 1), 6):
            if n % (i - 1) == 0 or n % (i + 1) == 0:
                return False
        return True
    return n == 2 or n == 3 or (n > 1 and n % 2 != 0 and n % 3 != 0 and _is_prime())

