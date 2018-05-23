#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./instruction_function/ux2_hexcmd.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#


import time
from .x2_hexcmd import X2HexCmd
from ..config import x2_config
from ..utils import crc16


class UX2HexCmd(X2HexCmd):
    def __init__(self, arm_port, fromid=x2_config.UXBUS_DEF_FROMID, toid=x2_config.UXBUS_DEF_TOID):
        super(UX2HexCmd, self).__init__()
        self.DB_FLG = '[ux2 hcmd] '
        self.arm_port = arm_port
        self.fromid = fromid
        self.toid = toid
        arm_port.flush(fromid, toid)
        self._has_err_warn = False

    @property
    def has_err_warn(self):
        return self._has_err_warn

    def check_xbus_proc(self, data, funcode=0):
        if data[3] & 0x40:
            self._has_err_warn = True
            return x2_config.UX2_ERR_CODE
        elif data[3] & 0x20:
            self._has_err_warn = True
            return x2_config.UX2_WAR_CODE
        else:
            self._has_err_warn = False
            return 0

    def send_pend(self, funcode, n, timeout):
        ret = [0] * (n + 1)
        times = int(timeout)
        ret[0] = x2_config.UX2_ERR_TOUT
        while times > 0:
            times -= 1
            rx_data = self.arm_port.read()
            if rx_data != -1 and len(rx_data) > 5:
                ret[0] = self.check_xbus_proc(rx_data)
                for i in range(n):
                    ret[i + 1] = rx_data[i + 4]
                return ret
            time.sleep(0.001)
        return ret

    def send_xbus(self, reg, txdata, num):
        send_data = bytes([self.fromid, self.toid, num + 1, reg])
        for i in range(num):
            send_data += bytes([txdata[i]])
        send_data += crc16.crc_modbus(send_data)
        self.arm_port.flush()
        return self.arm_port.write(send_data)
