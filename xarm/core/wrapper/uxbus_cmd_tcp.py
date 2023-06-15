#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import time
import struct
from ..utils import convert
from .uxbus_cmd import UxbusCmd, lock_require
from ..config.x_config import XCONF

STANDARD_MODBUS_TCP_PROTOCOL = 0x00
PRIVATE_MODBUS_TCP_PROTOCOL = 0x02
TRANSACTION_ID_MAX = 65535    # cmd序号 最大值


def debug_log_datas(datas, label=''):
    print('{}:'.format(label), end=' ')
    for i in range(len(datas)):
        print('{:x}'.format(datas[i]).zfill(2), end=' ')
        # print('0x{}'.format('{:x}'.format(datas[i]).zfill(2)), end=' ')
        # print(hex(rx_data[i]), end=',')
    print()


class UxbusCmdTcp(UxbusCmd):
    def __init__(self, arm_port, set_feedback_key_tranid=None):
        super(UxbusCmdTcp, self).__init__(set_feedback_key_tranid=set_feedback_key_tranid)
        self.arm_port = arm_port
        self._has_err_warn = False
        self._last_comm_time = time.monotonic()
        self._transaction_id = 1
        self._protocol_identifier = PRIVATE_MODBUS_TCP_PROTOCOL

    @property
    def has_err_warn(self):
        return self._has_err_warn

    @has_err_warn.setter
    def has_err_warn(self, value):
        self._has_err_warn = value

    @lock_require
    def set_protocol_identifier(self, protocol_identifier):
        if self._protocol_identifier != protocol_identifier:
            self._protocol_identifier = protocol_identifier
            print('change protocol identifier to {}'.format(self._protocol_identifier))
        return 0
    
    def get_protocol_identifier(self):
        return self._protocol_identifier
    
    def _get_trans_id(self):
        return self._transaction_id

    def check_protocol_header(self, data, t_trans_id, t_prot_id, t_unit_id):
        trans_id = convert.bytes_to_u16(data[0:2])
        prot_id = convert.bytes_to_u16(data[2:4])
        # length = convert.bytes_to_u16(data[4:6])
        unit_id = data[6]  # standard(unit_id), private(funcode)
        if trans_id != t_trans_id:
            return XCONF.UxbusState.ERR_NUM
        if prot_id != t_prot_id:
            return XCONF.UxbusState.ERR_PROT
        if unit_id != t_unit_id:
            return XCONF.UxbusState.ERR_FUN
        # if len(data) != length + 6:
        #     return XCONF.UxbusState.ERR_LENG
        return 0
    
    def check_private_protocol(self, data):
        state = data[7]
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
        return 0
    
    def send_modbus_request(self, unit_id, pdu_data, pdu_len, prot_id=-1, t_id=None):
        trans_id = self._transaction_id if t_id is None else t_id
        prot_id = self._protocol_identifier if prot_id < 0 else prot_id
        send_data = convert.u16_to_bytes(trans_id)
        send_data += convert.u16_to_bytes(prot_id)
        send_data += convert.u16_to_bytes(pdu_len + 1)
        send_data += bytes([unit_id])
        for i in range(pdu_len):
            send_data += bytes([pdu_data[i]])
        self.arm_port.flush()
        if self._debug:
            debug_log_datas(send_data, label='send({})'.format(unit_id))
        ret = self.arm_port.write(send_data)
        if ret != 0:
            return -1
        if t_id is None:
            self._transaction_id = self._transaction_id % TRANSACTION_ID_MAX + 1
        return trans_id
    
    def recv_modbus_response(self, t_unit_id, t_trans_id, num, timeout, t_prot_id=-1, ret_raw=False):
        prot_id = self._protocol_identifier if t_prot_id < 0 else t_prot_id
        ret = [0] * 320 if num == -1 else [0] * (num + 1)
        ret[0] = XCONF.UxbusState.ERR_TOUT
        expired = time.monotonic() + timeout
        while time.monotonic() < expired:
            remaining = expired - time.monotonic()
            rx_data = self.arm_port.read(remaining)
            if rx_data == -1:
                time.sleep(0.001)
                continue
            self._last_comm_time = time.monotonic()
            if self._debug:
                debug_log_datas(rx_data, label='recv({})'.format(t_unit_id))
            code = self.check_protocol_header(rx_data, t_trans_id, prot_id, t_unit_id)
            if code != 0:
                if code != XCONF.UxbusState.ERR_NUM:
                    ret[0] = code
                    return ret
                else:
                    continue
            if prot_id != STANDARD_MODBUS_TCP_PROTOCOL and not ret_raw:
                # Private Modbus TCP Protocol
                ret[0] = self.check_private_protocol(rx_data)
                num = convert.bytes_to_u16(rx_data[4:6]) - 2
                ret = ret[:num + 1] if len(ret) >= num + 1 else [ret[0]] * (num + 1)
                length = len(rx_data) - 8
                for i in range(num):
                    if i >= length:
                        break
                    ret[i + 1] = rx_data[i + 8]       
            else:
                # Standard Modbus TCP Protocol
                num = convert.bytes_to_u16(rx_data[4:6]) + 6
                ret = ret[:num + 1] if len(ret) >= num + 1 else [ret[0]] * (num + 1)
                length = len(rx_data)
                for i in range(num):
                    if i >= length:
                        break
                    ret[i + 1] = rx_data[i]
            return ret
        return ret

    # def send_hex_request(self, send_data):
    #     trans_id = int('0x' + str(send_data[0]) + str(send_data[1]), 16)
    #     data_str = b''
    #     for data in send_data:
    #         data_str += bytes.fromhex(data)
    #     send_data = data_str
    #     self.arm_port.flush()
    #     if self._debug:
    #         debug_log_datas(send_data, label='send')
    #     ret = self.arm_port.write(send_data)
    #     if ret != 0:
    #         return -1
    #     return trans_id

    # def recv_hex_request(self, t_trans_id, timeout, t_prot_id=-1):
    #     prot_id = 2
    #     expired = time.monotonic() + timeout
    #     while time.monotonic() < expired:
    #         remaining = expired - time.monotonic()
    #         rx_data = self.arm_port.read(remaining)
    #         if rx_data == -1:
    #             time.sleep(0.001)
    #             continue
    #         self._last_comm_time = time.monotonic()
    #         if self._debug:
    #             debug_log_datas(rx_data, label='recv')
    #         code = self.check_protocol_header(rx_data, t_trans_id, prot_id, rx_data[6])
    #         if code != 0:
    #             if code != XCONF.UxbusState.ERR_NUM:
    #                 return code
    #             else:
    #                 continue
    #         break
    #     if rx_data != -1 and len(rx_data) > 0:
    #         recv_datas = []
    #         for i in range(len(rx_data)):
    #             recv_datas.append('{:x}'.format(rx_data[i]).zfill(2))
    #         return recv_datas
    #     else:
    #         return rx_data if rx_data else 3

    ####################### Standard Modbus TCP API ########################
    @lock_require
    def __standard_modbus_tcp_request(self, pdu, unit_id=0x01):
        ret = self.send_modbus_request(unit_id, pdu, len(pdu), prot_id=STANDARD_MODBUS_TCP_PROTOCOL)
        if ret == -1:
            return XCONF.UxbusState.ERR_NOTTCP, b''
        ret = self.recv_modbus_response(unit_id, ret, -1, 10000, t_prot_id=STANDARD_MODBUS_TCP_PROTOCOL)
        code, recv_data = ret[0], bytes(ret[1:])
        if code == 0 and recv_data[7] == pdu[0] + 0x80:  # len(recv_data) == 9
            # print('request exception, exp={}, res={}'.format(recv_data[8], recv_data))
            return recv_data[8] + 0x80, recv_data
        # elif code != 0:
        #     print('recv timeout, len={}, res={}'.format(len(recv_data), recv_data))
        return code, recv_data
    
    def __read_bits(self, addr, quantity, func_code=0x01):
        assert func_code == 0x01 or func_code == 0x02
        pdu = struct.pack('>BHH', func_code, addr, quantity)
        code, res_data = self.__standard_modbus_tcp_request(pdu)
        if code == 0 and len(res_data) == 9 + (quantity + 7) // 8:
            return code, [(res_data[9 + i // 8] >> (i % 8) & 0x01) for i in range(quantity)]
        else:
            return code, res_data

    def __read_registers(self, addr, quantity, func_code=0x03, is_signed=False):
        assert func_code == 0x03 or func_code == 0x04
        pdu = struct.pack('>BHH', func_code, addr, quantity)
        code, res_data = self.__standard_modbus_tcp_request(pdu)
        if code == 0 and len(res_data) == 9 + quantity * 2:
            return 0, list(struct.unpack('>{}{}'.format(quantity, 'h' if is_signed else 'H'), res_data[9:]))
        else:
            return code, res_data
    
    def read_coil_bits(self, addr, quantity):
        """
        func_code: 0x01
        """
        return self.__read_bits(addr, quantity, func_code=0x01)

    def read_input_bits(self, addr, quantity):
        """
        func_code: 0x02
        """
        return self.__read_bits(addr, quantity, func_code=0x02)
    
    def read_holding_registers(self, addr, quantity, is_signed=False):
        """
        func_code: 0x03
        """
        return self.__read_registers(addr, quantity, func_code=0x03, is_signed=is_signed)

    def read_input_registers(self, addr, quantity, is_signed=False):
        """
        func_code: 0x04
        """
        return self.__read_registers(addr, quantity, func_code=0x04, is_signed=is_signed)
    
    def write_single_coil_bit(self, addr, bit_val):
        """
        func_code: 0x05
        """
        pdu = struct.pack('>BHH', 0x05, addr, 0xFF00 if bit_val else 0x0000)
        return self.__standard_modbus_tcp_request(pdu)[0]

    def write_single_holding_register(self, addr, reg_val):
        """
        func_code: 0x06
        """
        # pdu = struct.pack('>BHH', 0x06, addr, reg_val)
        pdu = struct.pack('>BH', 0x06, addr)
        pdu += convert.u16_to_bytes(reg_val)
        return self.__standard_modbus_tcp_request(pdu)[0]

    def write_multiple_coil_bits(self, addr, bits):
        """
        func_code: 0x0F
        """
        datas = [0] * ((len(bits) + 7) // 8)
        for i in range(len(bits)):
            if bits[i]:
                datas[i // 8] |= (1 << (i % 8))
        pdu = struct.pack('>BHHB{}B'.format(len(datas)), 0x0F, addr, len(bits), len(datas), *datas)
        return self.__standard_modbus_tcp_request(pdu)[0]

    def write_multiple_holding_registers(self, addr, regs):
        """
        func_code: 0x10
        """
        # pdu = struct.pack('>BHHB{}H'.format(len(regs)), 0x10, addr, len(regs), len(regs) * 2, *regs)
        pdu = struct.pack('>BHHB', 0x10, addr, len(regs), len(regs) * 2)
        pdu += convert.u16s_to_bytes(regs, len(regs))
        return self.__standard_modbus_tcp_request(pdu)[0]
    
    def mask_write_holding_register(self, addr, and_mask, or_mask):
        """
        func_code: 0x16
        """
        pdu = struct.pack('>BHHH', 0x16, addr, and_mask, or_mask)
        return self.__standard_modbus_tcp_request(pdu)[0]

    def write_and_read_holding_registers(self, r_addr, r_quantity, w_addr, w_regs, is_signed=False):
        """
        func_code: 0x17
        """
        # pdu = struct.pack('>BHHHHB{}{}'.format(len(w_regs), 'h' if w_signed else 'H'), 0x17, r_addr, r_quantity, w_addr, len(w_regs), len(w_regs) * 2, *w_regs)
        pdu = struct.pack('>BHHHHB', 0x17, r_addr, r_quantity, w_addr, len(w_regs), len(w_regs) * 2)
        pdu += convert.u16s_to_bytes(w_regs, len(w_regs))
        code, res_data = self.__standard_modbus_tcp_request(pdu)
        if code == 0 and len(res_data) == 9 + r_quantity * 2:
            return 0, struct.unpack('>{}{}'.format(r_quantity, 'h' if is_signed else 'H'), res_data[9:])
        else:
            return code, res_data