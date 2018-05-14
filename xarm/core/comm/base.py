#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>


import threading
import queue
import time
from ..utils.log import logger


class _Port(threading.Thread):
    def __init__(self, rxque_max):
        super(_Port, self).__init__()
        self.daemon = True
        self.rx_que = queue.Queue(rxque_max)
        self.write_lock = threading.Lock()
        self._connected = False
        self.com = None
        self.rx_parse = -1
        self.com_read = None
        self.com_write = None
        self.DB_FLG = 'port'
        self.port_type = ''
        self.buffer_size = 1

    @property
    def connected(self):
        return self._connected

    def run(self):
        self.recv_proc()

    def close(self):
        if self.connected:
            self.com.close()
            while self.connected:
                time.sleep(0.01)

    def flush(self, fromid=-1, toid=-1):
        if not self.connected:
            return -1
        while not(self.rx_que.empty()):
            self.rx_que.queue.clear()
        if self.rx_parse != -1:
            self.rx_parse.flush(fromid, toid)
        return 0

    def write(self, data):
        if not self.connected:
            return -1
        try:
            with self.write_lock:
                self.com_write(data)
            return 0
        except Exception as e:
            self._connected = False
            logger.error(self.DB_FLG + "{} send: {}".format(self.port_type, e))

    def read(self, timeout=None):
        if not self.connected:
            return -1
        if not self.rx_que.empty():
            buf = self.rx_que.get(timeout=timeout)
            return buf
        else:
            return -1

    def recv_proc(self):
        logger.debug(self.DB_FLG + 'arm {} thread start'.format(self.port_type))
        try:
            while self.connected:
                if self.port_type == 'cmd_socket':
                    rx_data = self.com_read(self.buffer_size)
                    if len(rx_data) == 0:
                        self._connected = False
                        break
                elif self.port_type == 'report_socket':
                    rx_data = self.com_read(self.buffer_size)
                    if len(rx_data) == 0:
                        self._connected = False
                        break
                elif self.port_type == 'serial':
                    rx_data = self.com_read(self.com.in_waiting or self.buffer_size)
                else:
                    break
                if -1 == self.rx_parse:
                    if self.rx_que.full():
                        self.rx_que.get()
                    self.rx_que.put(rx_data)
                else:
                    self.rx_parse.put(rx_data)
        except Exception as e:
            logger.error(e)
            if self.connected:
                self._connected = False
                self.com.close()
            logger.debug('disconnect')
        logger.debug(self.DB_FLG + '{} thread stop'.format(self.port_type))


