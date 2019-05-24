#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import time
from ..utils import crc16
from .uxbus_cmd import UxbusCmd
from ..config.x_config import XCONF


class UxbusCmdSer(UxbusCmd):
    def __init__(self, arm_port, fromid=XCONF.SerialConf.UXBUS_DEF_FROMID, toid=XCONF.SerialConf.UXBUS_DEF_TOID):
        super(UxbusCmdSer, self).__init__()
        self.arm_port = arm_port
        self.fromid = fromid
        self.toid = toid
        arm_port.flush(fromid, toid)
        self._has_err_warn = False

    @property
    def has_err_warn(self):
        return self._has_err_warn

    @has_err_warn.setter
    def has_err_warn(self, value):
        self._has_err_warn = value

    def check_xbus_prot(self, data, funcode=0):
        if data[3] & 0x40:
            self._has_err_warn = True
            return XCONF.UxbusState.ERR_CODE
        elif data[3] & 0x20:
            self._has_err_warn = True
            return XCONF.UxbusState.WAR_CODE
        else:
            self._has_err_warn = False
            return 0

    def send_pend(self, funcode, num, timeout):
        if num == -1:
            ret = [0] * 254
        else:
            ret = [0] * (num + 1)
        times = int(timeout)
        ret[0] = XCONF.UxbusState.ERR_TOUT
        while times > 0:
            times -= 1
            rx_data = self.arm_port.read()
            if rx_data != -1 and len(rx_data) > 5:
                ret[0] = self.check_xbus_prot(rx_data)
                if num == -1:
                    num = rx_data[2]
                for i in range(num):
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
