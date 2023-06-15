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


def debug_log_datas(datas, label=''):
    print('{}:'.format(label), end=' ')
    for i in range(len(datas)):
        print('{:x}'.format(datas[i]).zfill(2), end=' ')
        # print(hex(rx_data[i]), end=',')
    print()


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

    def set_protocol_identifier(self, protocol_identifier):
        return 0
    
    def get_protocol_identifier(self):
        return 0
    
    def check_protocol_header(self, data, t_trans_id, t_prot_id, t_unit_id):
        return 0
    
    def check_private_protocol(self, data):
        self._state_is_ready = not (data[3] & 0x10)
        if data[3] & 0x08:
            return XCONF.UxbusState.INVALID
        if data[3] & 0x40:
            self._has_err_warn = True
            return XCONF.UxbusState.ERR_CODE
        elif data[3] & 0x20:
            self._has_err_warn = True
            return XCONF.UxbusState.WAR_CODE
        else:
            self._has_err_warn = False
            return 0
    
    def send_modbus_request(self, reg, txdata, num, prot_id=-1, t_id=None):
        send_data = bytes([self.fromid, self.toid, num + 1, reg])
        for i in range(num):
            send_data += bytes([txdata[i]])
        send_data += crc16.crc_modbus(send_data)
        self.arm_port.flush()
        if self._debug:
            debug_log_datas(send_data, label='send')
        return self.arm_port.write(send_data)
    
    def recv_modbus_response(self, t_funcode, t_trans_id, num, timeout, t_prot_id=-1, ret_raw=False):
        ret = [0] * 254 if num == -1 else [0] * (num + 1)
        expired = time.monotonic() + timeout
        ret[0] = XCONF.UxbusState.ERR_TOUT
        while time.monotonic() < expired:
            remaining = expired - time.monotonic()
            rx_data = self.arm_port.read(remaining)
            if rx_data != -1 and len(rx_data) > 5:
                if self._debug:
                    debug_log_datas(rx_data, label='recv')
                ret[0] = self.check_private_protocol(rx_data)
                num = rx_data[2] if num == -1 else num
                length = len(rx_data) - 4
                for i in range(num):
                    if i >= length:
                        break
                    ret[i + 1] = rx_data[i + 4]
                return ret
            time.sleep(0.001)
        return ret
