#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import serial
from ..utils.log import logger
from .base import Port
from .uxbus_cmd_protocol import Ux2HexProtocol
from ..config.x_config import XCONF


class SerialPort(Port):
    def __init__(self, port, baud=XCONF.SerialConf.SERIAL_BAUD,
                 rxque_max=XCONF.SerialConf.UXBUS_RXQUE_MAX, protocol=XCONF.SerialConf.UX2_HEX_PROTOCOL):
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

            if protocol == XCONF.SerialConf.UX2_HEX_PROTOCOL:
                self.rx_parse = Ux2HexProtocol(self.rx_que,
                                               XCONF.SerialConf.UXBUS_DEF_FROMID,
                                               XCONF.SerialConf.UXBUS_DEF_TOID)
            self.com_read = self.com.read
            self.com_write = self.com.write
            self.start()
        except Exception as e:
            logger.info('{} connect {}:{} failed, {}'.format(self.port_type, port, baud, e))
            self._connected = False

