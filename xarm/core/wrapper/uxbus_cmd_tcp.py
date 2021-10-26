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


def debug_log_datas(datas, label=''):
    print('{}:'.format(label), end=' ')
    for i in range(len(datas)):
        print('{:x}'.format(datas[i]).zfill(2), end=' ')
        # print('0x{}'.format('{:x}'.format(datas[i]).zfill(2)), end=' ')
        # print(hex(rx_data[i]), end=',')
    print()


class UxbusCmdTcp(UxbusCmd):
    def __init__(self, arm_port):
        super(UxbusCmdTcp, self).__init__()
        self.arm_port = arm_port
        self.bus_flag = TX2_BUS_FLAG_MIN
        self.prot_flag = TX2_PROT_CON
        self._has_err_warn = False
        self._last_comm_time = time.time()

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
        self._state_is_ready = not (state & 0x10)
        if state & 0x08:
            return XCONF.UxbusState.INVALID
        if state & 0x40:
            self._has_err_warn = True
            return XCONF.UxbusState.ERR_CODE
        if state & 0x20:
            self._has_err_warn = True
            return XCONF.UxbusState.WAR_CODE
        self._has_err_warn = False
        if len(data) != length + 6:
            return XCONF.UxbusState.ERR_LENG
        # if state & 0x10:
        #     return XCONF.UxbusState.STATE_NOT_READY
        return 0

    def send_pend(self, funcode, num, timeout):
        ret = [0] * 320 if num == -1 else [0] * (num + 1)
        ret[0] = XCONF.UxbusState.ERR_TOUT
        expired = time.time() + timeout
        while time.time() < expired:
            remaining = expired - time.time()
            rx_data = self.arm_port.read(remaining)
            if rx_data != -1 and len(rx_data) > 7:
                self._last_comm_time = time.time()
                if self._debug:
                    debug_log_datas(rx_data, label='recv({})'.format(funcode))
                ret[0] = self.check_xbus_prot(rx_data, funcode)
                if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.STATE_NOT_READY]:
                    num = (convert.bytes_to_u16(rx_data[4:6]) - 2) if num == -1 else num
                    ret = ret[:num + 1] if len(ret) <= num + 1 else [ret[0]] * (num + 1)
                    length = len(rx_data) - 8
                    for i in range(num):
                        if i >= length:
                            break
                        ret[i + 1] = rx_data[i + 8]
                    return ret
                elif ret[0] != XCONF.UxbusState.ERR_NUM:
                    return ret
            else:
                time.sleep(0.001)
            # time.sleep(0.0005)
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
        if self._debug:
            debug_log_datas(send_data, label='send({})'.format(funcode))
        ret = self.arm_port.write(send_data)
        if ret != 0:
            return -1
        self.bus_flag += 1
        if self.bus_flag > TX2_BUS_FLAG_MAX:
            self.bus_flag = TX2_BUS_FLAG_MIN
        return 0
