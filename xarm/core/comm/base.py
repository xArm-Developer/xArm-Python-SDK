#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>


import threading
import queue
import functools
from ..utils.log import logger


class _Port(threading.Thread):
    def __init__(self, rxque_max):
        super(_Port, self).__init__()
        self.daemon = True
        self.rx_que = queue.Queue(rxque_max)
        # self.state = -1
        # self.com = None
        # self.rx_parse = -1
        # self.com_read = None
        # self.com_write = None
        # self.DB_FLG = 'port'
        # self.port_type = ''

    def is_ok(self):
        return self.state

    def run(self):
            self.recv_proc()

    def close(self):
        if 0 == self.state:
            self.com.close()
            self.state = -1

    def flush(self, fromid=-1, toid=-1):
        if 0 != self.state:
            return -1
        while not(self.rx_que.empty()):
            self.rx_que.queue.clear()
        if self.rx_parse != -1:
            self.rx_parse.flush(fromid, toid)
        return 0

    # def write(self, data):
    #     if 0 != self.state:
    #         return -1
    #     try:
    #         self.com_write(data)
    #         return 0
    #     except Exception as e:
    #         self.state = -1
    #         logger.error(self.DB_FLG + "{} send: {}".format(self.port_type, e))
    #         return -1

    def read(self, timeout=None):
        if 0 != self.state:
            return -1
        if not self.rx_que.empty():
            buf = self.rx_que.get(timeout=timeout)
            return buf
        else:
            return -1

    def recv_proc(self):
        logger.debug(self.DB_FLG + 'arm {} thread start'.format(self.port_type))
        try:
            while 0 == self.state:
                if self.port_type == 'socket':
                    rx_data = self.com_read(1024)
                    if len(rx_data) == 0:
                        self.state = -1
                        break
                else:
                    rx_data = self.com_read(self.com.in_waiting or 1)
                if -1 == self.rx_parse:
                    if self.rx_que.full():
                        self.rx_que.get()
                    self.rx_que.put(rx_data)
                else:
                    self.rx_parse.put(rx_data)
        except Exception as e:
            logger.error(e)
            self.state = -1
            self.com.close()
        logger.debug(self.DB_FLG + '{} thread stop'.format(self.port_type))


