#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./communat_port/serial_port.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#

import serial
import threading
import queue
from ..config import x2_config
from ..protocol.ux2_hex_protocol import UX2HexProtocol
from ..utils.log import logger
from .base import _Port


class SerialPort(_Port):
    def __init__(self, port, buad=x2_config.CTSER_COM_BAUD,
                 rxque_max=x2_config.UXBUS_RXQUE_MAX, protocol=x2_config.UX2_HEX_PROTOCOL):
        try:
            self.com = serial.Serial(port=port, baudrate=buad)
            if not(self.com.isOpen()):
                self.state = -1
                return

            self.state = 0
            self.DB_FLG = '[ser proc] '

            # self.rx_que = queue.Queue(x2_config.UXBUS_RXQUE_MAX)
            # threading.Thread.__init__(self)
            # self.daemon = True

            self.port_type = 'serial'
            super(SerialPort, self).__init__(rxque_max)
            if x2_config.UX2_HEX_PROTOCOL == protocol:
                self.rx_parse = UX2HexProtocol(self.rx_que, x2_config.UXBUS_DEF_FROMID, x2_config.UXBUS_DEF_TOID)
            else:
                self.rx_parse = -1
            self.com_read = self.com.read
            self.com_write = self.com.write

            self.start()
        except Exception as e:
            logger.error(e)
            self.state = -1

    def write(self, data):
        if 0 != self.state:
            return -1
        try:
            self.com.write(data)
            return 0
        except:
            self.state = -1
            logger.error(self.DB_FLG + " serial send")
            return -1

    # def is_ok(self):
    #     return self.state
    #
    # def run(self):
    #         self.recv_proc()
    #
    # def close(self):
    #     if 0 == self.state:
    #         self.com.close()
    #         self.state = -1
    #
    # def flush(self, fromid=-1, toid=-1):
    #     if 0 != self.state:
    #         return -1
    #     while not(self.rx_que.empty()):
    #         self.rx_que.get()
    #     if self.rx_parse != -1:
    #         self.rx_parse.flush(fromid, toid)
    #     return 0
    #
    # def write(self, data):
    #     if 0 != self.state:
    #         return -1
    #     try:
    #         self.com.write(data)
    #         return 0
    #     except:
    #         self.state = -1
    #         logger.debug(self.DB_FLG + "Error: serial send")
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
    #     logger.debug(self.DB_FLG + 'arm serial thread start')
    #     try:
    #         while 0 == self.state:
    #             rxch = self.com.read(1)
    #             # print(rxch)
    #             if -1 == self.rx_parse:
    #                 if self.rx_que.full():
    #                     self.rx_que.get()
    #                 self.rx_que.put(rxch)
    #             else:
    #                 self.rx_parse.put(rxch, 1)
    #     except:
    #         self.state = -1
    #     logger.debug(self.DB_FLG + 'uxserial thread stop')
