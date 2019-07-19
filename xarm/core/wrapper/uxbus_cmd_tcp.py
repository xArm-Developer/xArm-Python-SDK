#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import time
from ..utils import convert
from .uxbus_cmd import UxbusCmd
from ..config.x_config import XCONF


TX2_PROT_CON = 2  # tcp cmd prot
TX2_PROT_HEAT = 1  # tcp heat prot
TX2_BUS_FLAG_MIN = 1  # cmd序号 起始值
TX2_BUS_FLAG_MAX = 5000  # cmd序号 最大值


class UxbusCmdTcp(UxbusCmd):
    def __init__(self, arm_port):
        super(UxbusCmdTcp, self).__init__()
        self.arm_port = arm_port
        self.bus_flag = TX2_BUS_FLAG_MIN
        self.prot_flag = TX2_PROT_CON
        self._has_err_warn = False

    @property
    def has_err_warn(self):
        return self._has_err_warn

    @has_err_warn.setter
    def has_err_warn(self, value):
        self._has_err_warn = value

    def check_xbus_prot(self, data, funcode):
        num = convert.bytes_to_u16(data[0:2])
        prot = convert.bytes_to_u16(data[2:4])
        length = convert.bytes_to_u16(data[4:6])
        fun = data[6]
        state = data[7]

        bus_flag = self.bus_flag
        if bus_flag == TX2_BUS_FLAG_MIN:
            bus_flag = TX2_BUS_FLAG_MAX
        else:
            bus_flag -= 1
        if num != bus_flag:
            return XCONF.UxbusState.ERR_NUM
        if prot != TX2_PROT_CON:
            return XCONF.UxbusState.ERR_PROT
        if fun != funcode:
            return XCONF.UxbusState.ERR_FUN
        if state & 0x40:
            self._has_err_warn = True
            return XCONF.UxbusState.ERR_CODE
        if state & 0x20:
            self._has_err_warn = True
            return XCONF.UxbusState.WAR_CODE
        if len(data) != length + 6:
            return XCONF.UxbusState.ERR_LENG
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
            if rx_data != -1 and len(rx_data) > 7:
                # print('recv:',  end=' ')
                # for i in range(len(rx_data)):
                #     print(hex(rx_data[i]), end=',')
                # print()
                ret[0] = self.check_xbus_prot(rx_data, funcode)
                if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
                    if num == -1:
                        num = rx_data[5] - 2
                        ret1 = [ret[0]] * (num + 1)
                        for i in range(num):
                            ret1[i + 1] = rx_data[i + 8]
                        return ret1
                    else:
                        for i in range(num):
                            ret[i + 1] = rx_data[i + 8]
                return ret
            time.sleep(0.001)
        return ret

    def send_xbus(self, funcode, datas, num):
        send_data = convert.u16_to_bytes(self.bus_flag)
        send_data += convert.u16_to_bytes(self.prot_flag)
        send_data += convert.u16_to_bytes(num + 1)
        send_data += bytes([funcode])
        if type(datas) == str:
            send_data += datas.encode()
        else:
            for i in range(num):
                send_data += bytes([datas[i]])
        self.arm_port.flush()
        # print('send:', end=' ')
        # for i in range(len(send_data)):
        #     print(hex(send_data[i]), end=',')
        # print()
        ret = self.arm_port.write(send_data)
        if ret != 0:
            return -1
        self.bus_flag += 1
        if self.bus_flag > TX2_BUS_FLAG_MAX:
            self.bus_flag = TX2_BUS_FLAG_MIN
        return 0
