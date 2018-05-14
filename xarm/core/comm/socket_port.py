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

    def run(self):
        logger.debug('heart beat thread start')
        heat_data = bytes([0, 0, 0, 1, 0, 2, 0, 0])

        while True:
            if self.sock_class.write(heat_data) == -1:
                break
            time.sleep(1)
        logger.debug('heart beat thread exit')


class SocketPort(_Port):
    def __init__(self, server_ip, server_port, rxque_max=x2_config.TCP_RX_QUE_MAX, heartbeat=False, buffer_size=1024):
        super(SocketPort, self).__init__(rxque_max)
        try:
            socket.setdefaulttimeout(1)
            self.com = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.com.setblocking(True)
            self.com.connect((server_ip, server_port))

            self.DB_FLG = '[tcp proc] '
            self._connected = True
            self.buffer_size = buffer_size

            if server_port == x2_config.SERVER_PORT:
                self.port_type = 'cmd_socket'
                # self.com.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, 5)
            else:
                self.port_type = 'report_socket'
            self.rx_parse = -1
            self.com_read = self.com.recv
            self.com_write = self.com.send
            self.write_lock = threading.Lock()
            self.start()
            if heartbeat:
                heat_proc = HeartBeat(self)
                heat_proc.start()
        except Exception as e:
            logger.error(e)
            self._connected = False

