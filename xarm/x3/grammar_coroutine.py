#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import asyncio
from ..core.utils.log import logger


class CoroutineObject(object):
    @asyncio.coroutine
    def _asyncio_loop_func(self):
        logger.debug('asyncio thread start ...')
        while self.connected:
            yield from asyncio.sleep(0.001)
        logger.debug('asyncio thread exit ...')
    
    @staticmethod
    @asyncio.coroutine
    def _async_run_callback(callback, msg):
        yield from callback(msg)