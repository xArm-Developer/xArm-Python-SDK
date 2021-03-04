#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import queue
import socket
import threading
import time
from ..utils.log import logger
from .base import Port
from ..config.x_config import XCONF


class HeartBeatThread(threading.Thread):
    def __init__(self, sock_class):
        threading.Thread.__init__(self)
        self.sock_class = sock_class
        self.daemon = True

    def run(self):
        logger.debug('{} heartbeat thread start'.format(self.sock_class.port_type))
        heat_data = bytes([0, 0, 0, 1, 0, 2, 0, 0])

        while self.sock_class.connected:
            if self.sock_class.write(heat_data) == -1:
                break
            time.sleep(1)
        logger.debug('{} heartbeat thread had stopped'.format(self.sock_class.port_type))


class SocketPort(Port):
    def __init__(self, server_ip, server_port, rxque_max=XCONF.SocketConf.TCP_RX_QUE_MAX, heartbeat=False,
                 buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE):
        super(SocketPort, self).__init__(rxque_max)
        if server_port == XCONF.SocketConf.TCP_CONTROL_PORT:
            self.port_type = 'main-socket'
            # self.com.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, 5)
        else:
            self.port_type = 'report-socket'
        try:
            socket.setdefaulttimeout(1)
            self.com = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.com.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # self.com.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            # self.com.setsockopt(socket.SOL_TCP, socket.TCP_KEEPIDLE, 30)
            # self.com.setsockopt(socket.SOL_TCP, socket.TCP_KEEPINTVL, 10)
            # self.com.setsockopt(socket.SOL_TCP, socket.TCP_KEEPCNT, 3)
            self.com.setblocking(True)
            self.com.settimeout(1)
            self.com.connect((server_ip, server_port))
            logger.info('{} connect {} success'.format(self.port_type, server_ip))
            # logger.info('{} connect {}:{} success'.format(self.port_type, server_ip, server_port))

            self._connected = True
            self.buffer_size = buffer_size
            # time.sleep(1)

            self.com_read = self.com.recv
            self.com_write = self.com.send
            self.write_lock = threading.Lock()
            self.start()
            if heartbeat:
                self.heartbeat_thread = HeartBeatThread(self)
                self.heartbeat_thread.start()
        except Exception as e:
            logger.info('{} connect {} failed, {}'.format(self.port_type, server_ip, e))
            # logger.error('{} connect {}:{} failed, {}'.format(self.port_type, server_ip, server_port, e))
            self._connected = False

