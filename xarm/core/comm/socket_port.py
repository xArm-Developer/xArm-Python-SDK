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
        logger.info('{} heartbeat thread start'.format(self.sock_class.port_type))
        heat_data = bytes([0, 0, 0, 1, 0, 2, 0, 0])

        while self.sock_class.connected:
            if self.sock_class.write(heat_data) == -1:
                break
            time.sleep(1)
        logger.info('{} heartbeat thread had stopped'.format(self.sock_class.port_type))


class SocketPort(_Port):
    def __init__(self, server_ip, server_port, rxque_max=x2_config.TCP_RX_QUE_MAX, heartbeat=False, buffer_size=1024):
        super(SocketPort, self).__init__(rxque_max)
        if server_port == x2_config.SERVER_PORT:
            self.port_type = 'main-socket'
            # self.com.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, 5)
        else:
            self.port_type = 'report-socket'
        try:
            socket.setdefaulttimeout(1)
            self.com = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.com.setblocking(True)
            self.com.settimeout(1)
            self.com.connect((server_ip, server_port))
            logger.info('{} connect {}:{} success'.format(self.port_type, server_ip, server_port))

            self._connected = True
            self.buffer_size = buffer_size

            self.rx_parse = -1
            self.com_read = self.com.recv
            self.com_write = self.com.send
            self.write_lock = threading.Lock()
            self.start()
            if heartbeat:
                self.heartbeat_thread = HeartBeat(self)
                self.heartbeat_thread.start()
        except Exception as e:
            logger.info('{} connect {}:{} failed, {}'.format(self.port_type, server_ip, server_port, e))
            self._connected = False

