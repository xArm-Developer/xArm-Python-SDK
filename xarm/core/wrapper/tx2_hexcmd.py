#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./instruction_function/tx2_hexcmd.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#

import time
from .x2_hexcmd import X2HexCmd
from ..utils import convert
from ..config import x2_config

TX2_PROT_CON     = 2    # tcp cmd prot
TX2_PROT_HEAT    = 1    # tcp heat prot
TX2_BUS_FLAG_MIN = 1    # cmd序号 起始值
TX2_BUS_FLAG_MAX = 5000 # cmd序号 最大值


class TX2HexCmd(X2HexCmd):
    def __init__(self, arm_port):
        super(TX2HexCmd, self).__init__()
        self.DB_FLG = '[ux2 hcmd] '
        self.arm_port = arm_port
        self.bus_flag = TX2_BUS_FLAG_MIN
        self.prot_flag = TX2_PROT_CON

    def check_xbus_proc(self, data, funcode):
        num = convert.bytes_to_u16(data[0:2])
        prot = convert.bytes_to_u16(data[2:4])
        # leng = convert.bytes_to_u16(data[4:6])
        fun = data[6]
        state = data[7]

        bus_flag = self.bus_flag
        if bus_flag == TX2_BUS_FLAG_MIN:
            bus_flag = TX2_BUS_FLAG_MAX
        else:
            bus_flag -= 1
        if num != bus_flag:
            return x2_config.UX2_ERR_NUM
        if prot != TX2_PROT_CON:
            return x2_config.UX2_ERR_PROT
        if fun != funcode:
            return x2_config.UX2_ERR_FUN
        if state & 0x40:
            return x2_config.UX2_ERR_CODE
        elif state & 0x20:
            return x2_config.UX2_WAR_CODE
        return 0

    def send_pend(self, funcode, n, timeout):
        ret = [0] * (n + 1)
        times = int(timeout)
        ret[0] = x2_config.UX2_ERR_TOUT
        while times > 0:
            times -= 1
            rx_data = self.arm_port.read()
            if rx_data != -1 and len(rx_data) > 7:
                ret[0] = self.check_xbus_proc(rx_data, funcode)
                for i in range(n):
                    ret[i + 1] = rx_data[i + 8]
                return ret
            time.sleep(0.001)
        return ret

    def send_xbus(self, funcode, datas, n):
        send_data = convert.u16_to_bytes(self.bus_flag)
        send_data += convert.u16_to_bytes(self.prot_flag)
        send_data += convert.u16_to_bytes(n + 1)
        send_data += bytes([funcode])
        for i in range(n):
            send_data += bytes([datas[i]])

        self.arm_port.flush()
        ret = self.arm_port.write(send_data)
        if 0 != ret:
            return -1
        self.bus_flag += 1
        if self.bus_flag > TX2_BUS_FLAG_MAX:
            self.bus_flag = TX2_BUS_FLAG_MIN
        return 0
