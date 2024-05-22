# !/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import sys
import time
import socket
import struct
import logging
import threading

logger = logging.Logger('modbus_tcp')
logger_fmt = '[%(levelname)s][%(asctime)s][%(filename)s:%(lineno)d] - - %(message)s'
logger_date_fmt = '%Y-%m-%d %H:%M:%S'
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setLevel(logging.DEBUG)
stream_handler.setFormatter(logging.Formatter(logger_fmt, logger_date_fmt))
logger.addHandler(stream_handler)
logger.setLevel(logging.INFO)


class ModbusTcpClient(object):
    def __init__(self, ip, port=502, unit_id=0x01):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(True)
        self.sock.connect((ip, port))
        self._transaction_id = 0
        self._protocol_id = 0x00
        self._unit_id = unit_id
        self._func_code = 0x00
        self._lock = threading.Lock()
    
    def __wait_to_response(self, transaction_id=None, unit_id=None, func_code=None, timeout=3):
        expired = time.monotonic() + timeout
        recv_data = b''
        length = 0
        code = -3  # TIMEOUT
        send_transaction_id = transaction_id if transaction_id is not None else self._transaction_id
        send_unit_id = unit_id if unit_id is not None else self._unit_id
        send_func_code = func_code if func_code is not None else self._func_code
        while time.monotonic() < expired:
            if len(recv_data) < 7:
                recv_data += self.sock.recv(7 - len(recv_data))
            if len(recv_data) < 7:
                continue
            if length == 0:
                length = struct.unpack('>H', recv_data[4:6])[0]
            if len(recv_data) < length + 6:
                recv_data += self.sock.recv(length + 6 - len(recv_data))
            if len(recv_data) < length + 6:
                continue
            transaction_id = struct.unpack('>H', recv_data[0:2])[0]
            protocol_id = struct.unpack('>H', recv_data[2:4])[0]
            unit_id = recv_data[6]
            func_code = recv_data[7]
            if transaction_id != send_transaction_id:
                logger.warning('Receive a reply with a mismatched transaction id (S: {}, R: {}), discard it and continue waiting.'.format(send_transaction_id, transaction_id))
                length = 0
                recv_data = b''
                continue
            elif protocol_id != self._protocol_id:
                logger.warning('Receive a reply with a mismatched protocol id (S: {}, R: {}), discard it and continue waiting.'.format(self._protocol_id, protocol_id))
                length = 0
                recv_data = b''
                continue
            elif unit_id != send_unit_id:
                logger.warning('Receive a reply with a mismatched unit id (S: {}, R: {}), discard it and continue waiting.'.format(send_unit_id, unit_id))
                length = 0
                recv_data = b''
                continue
            elif func_code != send_func_code and func_code != send_func_code + 0x80:
                logger.warning('Receive a reply with a mismatched func code (S: {}, R: {}), discard it and continue waiting.'.format(send_func_code, func_code))
                length = 0
                recv_data = b''
                continue
            else:
                code = 0
                break
        if code == 0 and len(recv_data) == 9:
            logger.error('modbus tcp data exception, exp={}, res={}'.format(recv_data[8], recv_data))
            return recv_data[8], recv_data
        elif code != 0:
            logger.error('recv timeout, len={}, res={}'.format(len(recv_data), recv_data))
        return code, recv_data

    def __pack_to_send(self, pdu_data, unit_id=None):
        self._transaction_id = self._transaction_id % 65535 + 1
        unit_id = unit_id if unit_id is not None else self._unit_id
        data = struct.pack('>HHHB', self._transaction_id, self._protocol_id, len(pdu_data) + 1, unit_id)
        data += pdu_data
        self.sock.send(data)

    def __request(self, pdu, unit_id=None):
        with self._lock:
            self._func_code = pdu[0]
            self.__pack_to_send(pdu)
            return self.__wait_to_response(unit_id=unit_id, func_code=pdu[0])
    
    def __read_bits(self, addr, quantity, func_code=0x01):
        assert func_code == 0x01 or func_code == 0x02
        pdu = struct.pack('>BHH', func_code, addr, quantity)
        code, res_data = self.__request(pdu)
        if code == 0 and len(res_data) == 9 + (quantity + 7) // 8:
            return code, [(res_data[9 + i // 8] >> (i % 8) & 0x01) for i in range(quantity)]
        else:
            return code, res_data

    def __read_registers(self, addr, quantity, func_code=0x03, signed=False):
        assert func_code == 0x03 or func_code == 0x04
        pdu = struct.pack('>BHH', func_code, addr, quantity)
        code, res_data = self.__request(pdu)
        if code == 0 and len(res_data) == 9 + quantity * 2:
            return 0, list(struct.unpack('>{}{}'.format(quantity, 'h' if signed else 'H'), res_data[9:]))
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
    
    def read_holding_registers(self, addr, quantity, signed=False):
        """
        func_code: 0x03
        """
        return self.__read_registers(addr, quantity, func_code=0x03, signed=signed)

    def read_input_registers(self, addr, quantity, signed=False):
        """
        func_code: 0x04
        """
        return self.__read_registers(addr, quantity, func_code=0x04, signed=signed)
    
    def write_single_coil_bit(self, addr, on):
        """
        func_code: 0x05
        """
        pdu = struct.pack('>BHH', 0x05, addr, 0xFF00 if on else 0x0000)
        return self.__request(pdu)[0]

    def write_single_holding_register(self, addr, reg_val):
        """
        func_code: 0x06
        """
        pdu = struct.pack('>BHH', 0x06, addr, reg_val)
        return self.__request(pdu)[0]

    def write_multiple_coil_bits(self, addr, bits):
        """
        func_code: 0x0F
        """
        datas = [0] * ((len(bits) + 7) // 8)
        for i in range(len(bits)):
            if bits[i]:
                datas[i // 8] |= (1 << (i % 8))
        pdu = struct.pack('>BHHB{}B'.format(len(datas)), 0x0F, addr, len(bits), len(datas), *datas)
        return self.__request(pdu)[0]

    def write_multiple_holding_registers(self, addr, regs):
        """
        func_code: 0x10
        """
        pdu = struct.pack('>BHHB{}H'.format(len(regs)), 0x10, addr, len(regs), len(regs) * 2, *regs)
        return self.__request(pdu)[0]
    
    def mask_write_holding_register(self, addr, and_mask, or_mask):
        """
        func_code: 0x16
        """
        pdu = struct.pack('>BHHH', 0x16, addr, and_mask, or_mask)
        return self.__request(pdu)[0]

    def write_and_read_holding_registers(self, r_addr, r_quantity, w_addr, w_regs, r_signed=False, w_signed=False):
        """
        func_code: 0x17
        """
        pdu = struct.pack('>BHHHHB{}{}'.format(len(w_regs), 'h' if w_signed else 'H'), 0x17, r_addr, r_quantity, w_addr, len(w_regs), len(w_regs) * 2, *w_regs)
        code, res_data = self.__request(pdu)
        if code == 0 and len(res_data) == 9 + r_quantity * 2:
            return 0, struct.unpack('>{}{}'.format(r_quantity, 'h' if r_signed else 'H'), res_data[9:])
        else:
            return code, res_data