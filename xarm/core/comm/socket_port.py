#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./communat_port/socket_port.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#

import queue
import socket
import threading
import time
from ..utils.log import logger
from ..config import x2_config
from .base import _Port


class HeartBeat(threading.Thread):
    def __init__(self, sock_class):
        threading.Thread.__init__(self)
        self.sock_class = sock_class
        self.daemon = True
        self.DB_FLG = '[heatproc] '

    def run(self):
        logger.debug(self.DB_FLG + 'heat proc thread start')
        heat_data = bytes([0, 0, 0, 1, 0, 2, 0, 0])

        while True:
            if self.sock_class.write(heat_data) == -1:
                break
            time.sleep(1)
        logger.debug(self.DB_FLG + "heat proc exit")


class SocketPort(_Port):
    def __init__(self, server_ip, server_port, rxque_max=x2_config.TCP_RX_QUE_MAX, heat=0):
        try:
            socket.setdefaulttimeout(1)
            self.com = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.com.setblocking(True)
            self.com.connect((server_ip, server_port))

            self.DB_FLG = '[tcp proc] '
            self.state = 0

            # self.rx_que = queue.Queue(rxque_max)
            # threading.Thread.__init__(self)
            # self.daemon = True

            self.port_type = 'socket'
            super(SocketPort, self).__init__(rxque_max)
            self.rx_parse = -1
            self.com_read = self.com.recv
            self.com_write = self.com.send
            self.write_lock = threading.Lock()
            self.start()
            if heat:
                heat_proc = HeartBeat(self)
                heat_proc.start()
        except Exception as e:
            logger.error(e)
            self.state = -1

    def write(self, data):
        if 0 != self.state:
            return -1
        try:
            with self.write_lock:
                self.com_write(data)
            return 0
        except Exception as e:
            self.state = -1
            logger.error(self.DB_FLG + "{} send: {}".format(self.port_type, e))
            return -1

    # def is_ok(self):
    #     return self.state
    #
    # def run(self):
    #     self.recv_proc()
    #
    # def close(self):
    #     if 0 == self.state:
    #         self.fp.close()
    #         self.state = -1
    #
    # def flush(self):
    #     if 0 != self.state:
    #         return -1
    #     while not self.rx_que.empty():
    #         self.rx_que.queue.clear()
    #     return 0
    #
    # def write(self, data):
    #     if 0 != self.state:
    #         return -1
    #     try:
    #         with write_lock:
    #             self.fp.send(data)
    #         return 0
    #     except:
    #         self.state = -1
    #         logger.debug(self.DB_FLG + "Error: socket send")
    #         return -1
    #
    # def read(self, timeout=None):
    #     if 0 != self.state:
    #         return -1
    #     if not self.rx_que.empty():
    #         buf = self.rx_que.get(timeout=timeout)
    #         return buf
    #     else:
    #         return -1
    #
    # def recv_proc(self):
    #     logger.debug(self.DB_FLG + 'arm socket thread start')
    #     try:
    #         while 0 == self.state:
    #             rx_data = self.fp.recv(1024)
    #             if 0 == len(rx_data):
    #                 self.state = 0
    #                 break
    #             if self.rx_que.full():
    #                 self.rx_que.get()
    #             self.rx_que.put(rx_data)
    #     except:
    #         self.state = -1
    #     logger.debug(self.DB_FLG + 'recv_proc exit')
