#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./common_modules/common_type.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#

import struct
from functools import partial
from .bytes_data import BytesData


# def fp32_to_bytes(data, is_big_endian=False):
#     """默认小端字节序"""
#     return bytes(struct.pack('>f' if is_big_endian else '<f', data))
fp32_to_bytes = BytesData.from_fp32


# def fp32s_to_bytes(data, n):
#     """小端字节序"""
#     assert n > 0
#     ret = fp32_to_bytes(data[0])
#     for i in range(1, n):
#         ret += fp32_to_bytes(data[i])
#     return ret
fp32s_to_bytes = BytesData.from_fp32_list


def int32_to_bytes(data, is_big_endian=False):
    """默认小端字节序"""
    # return bytes(struct.pack('>i' if is_big_endian else '<i', data))
    return BytesData.from_s32(data, is_big_endian=is_big_endian)


# def int32s_to_bytes(data, n):
#     """小端字节序"""
#     assert n > 0
#     ret = int32_to_bytes(data[0])
#     for i in range(1, n):
#         ret += int32_to_bytes(data[i])
#     return ret
int32s_to_bytes = partial(BytesData.from_s32_list, is_big_endian=False)


# def bytes_to_fp32(data):
#     """小端字节序"""
#     ret = struct.unpack('<f', bytes(data[:4]))
#     return ret[0]
bytes_to_fp32 = BytesData.to_fp32


# def bytes_to_fp32s(data, n):
#     """小端字节序"""
#     ret = [0] * n
#     for i in range(n):
#         ret[i] = bytes_to_fp32(data[i * 4:i * 4 + 4])
#     return ret
bytes_to_fp32s = BytesData.to_fp32_list


# def u16_to_bytes(data):
#     """大端字节序"""
#     bts = bytes([data // 256 % 256])
#     bts += bytes([data % 256])
#     return bts
u16_to_bytes = BytesData.from_u16


# def u16s_to_bytes(data, num):
#     """大端字节序"""
#     bts = b''
#     if num != 0:
#         bts = u16_to_bytes(data[0])
#         for i in range(1, num):
#             bts += u16_to_bytes(data[i])
#     return bts
u16s_to_bytes = BytesData.from_u16_list


# def bytes_to_u16(data):
#     """大端字节序"""
#     data_u16 = data[0] << 8 | data[1]
#     return data_u16
bytes_to_u16 = BytesData.to_u16


# def bytes_to_u16s(data, n):
#     """大端字节序"""
#     ret = [0] * n
#     for i in range(n):
#         ret[i] = bytes_to_u16(data[i * 2: i * 2 + 2])
#     return ret
bytes_to_u16s = BytesData.to_u16_list


# def bytes_to_16s(data, n):
#     """大端字节序"""
#     ret = [0] * n
#     for i in range(n):
#         ret[i] = struct.unpack('>h', bytes(data[i * 2: i * 2 + 2]))[0]
#     return ret
bytes_to_16s = BytesData.to_s16_list


# def bytes_to_u32(data):
#     """大端字节序"""
#     data_u32 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
#     return data_u32
bytes_to_u32 = BytesData.to_u32

# def bytes_to_u32s(data, n):
#     """大端字节序"""
#     ret = [0] * n
#     for i in range(n):
#         ret[i] = bytes_to_u32(data[i * 4: i * 4 + 4])
#     return ret
bytes_to_u32s = BytesData.to_u32_list


# def bytes_to_u64(data):
#     """大端字节序"""
#     data_u64 = data[0] << 56 | data[1] << 48 | data[2] << 40 | data[3] << 32 | data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]
#     return data_u64
bytes_to_u64 = BytesData.to_u64


# def bytes_to_num32(data, fmt='>i'):
#     ret = struct.unpack(fmt, bytes(data[:4]))
#     return ret[0]
bytes_to_num32 = BytesData.to_num


# def bytes_to_long_big(data):
#     """大端字节序"""
#     return bytes_to_int32(data, is_big_endian=True)
bytes_to_long_big = BytesData.to_s32

# def bytes_to_int32(data, is_big_endian=True):
#     return bytes_to_num32(data, fmt='>i' if is_big_endian else '<i')
#     # return int.from_bytes(data, byteorder='big', signed=True)
bytes_to_int32 = BytesData.to_s32