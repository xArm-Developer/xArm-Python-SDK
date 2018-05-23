#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./common_modules/common_type.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#

import struct


def fp32_to_bytes(data):
    return bytes(struct.pack("f", data))


def bytes_to_fp32(data):
    byte = bytes([data[0]])
    byte += bytes([data[1]])
    byte += bytes([data[2]])
    byte += bytes([data[3]])
    return struct.unpack("f", byte)


def fp32s_to_bytes(data, n):
    assert n > 0
    ret = fp32_to_bytes(data[0])
    for i in range(1, n):
        ret += fp32_to_bytes(data[i])
    return ret


def bytes_to_fp32s(data, n):
    ret = [0] * n
    for i in range(n):
        ret[i] = bytes_to_fp32(data[i * 4: i * 4 + 4])
    return ret


def u16_to_bytes(data):
    bts = bytes([data // 256 % 256])
    bts += bytes([data % 256])
    return bts


def bytes_to_u16(data):
    data_u16 = data[0] << 8 | data[1]
    return data_u16


def bytes_to_u16s(data, n):
    ret = [0] * n
    for i in range(n):
        ret[i] = bytes_to_u16(data[i * 2: i * 2 + 2])
    return ret


def bytes_to_u32(data):
    data_u32 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
    return data_u32


def bytes_to_long_big(data):
    byte = bytes([data[0]])
    byte += bytes([data[1]])
    byte += bytes([data[2]])
    byte += bytes([data[3]])
    return struct.unpack(">l", byte)
