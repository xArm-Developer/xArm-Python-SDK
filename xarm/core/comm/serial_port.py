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
    def __init__(self, port, baud=x2_config.CTSER_COM_BAUD,
                 rxque_max=x2_config.UXBUS_RXQUE_MAX, protocol=x2_config.UX2_HEX_PROTOCOL):
        super(SerialPort, self).__init__(rxque_max)
        self.port_type = 'main-serial'
        try:
            self.com = serial.Serial(port=port, baudrate=baud)
            if not self.com.isOpen():
                self._connected = False
                raise Exception('serial is not open')
            logger.info('{} connect {}:{} success'.format(self.port_type, port, baud))

            self._connected = True

            self.buffer_size = 1

            if x2_config.UX2_HEX_PROTOCOL == protocol:
                self.rx_parse = UX2HexProtocol(self.rx_que, x2_config.UXBUS_DEF_FROMID, x2_config.UXBUS_DEF_TOID)
            else:
                self.rx_parse = -1
            self.com_read = self.com.read
            self.com_write = self.com.write
            self.start()
        except Exception as e:
            logger.info('{} connect {}:{} failed, {}'.format(self.port_type, port, baud, e))
            self._connected = False

