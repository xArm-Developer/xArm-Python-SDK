#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2025, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import struct


class BytesData:
    ################################ common #################################
    @staticmethod
    def from_num(val, fmt):
        return bytes(struct.pack(fmt, val))
    
    # @staticmethod
    # def from_uint(val, size=4, is_big_endian=True):
    #     if is_big_endian:
    #         return bytes([val >> (8 * (size - 1 - i)) & 0xFF for i in range(size)])
    #         # return bytes([val >> 24 & 0xFF, val >> 16 & 0xFF, val >> 8 & 0xFF, val & 0xFF])
    #     else:
    #         return bytes([val >> (8 * i) & 0xFF for i in range(4)])
    #         # return bytes([val & 0xFF, val >> 8 & 0xFF, val >> 16 & 0xFF, val >> 24 & 0xFF])

    @staticmethod
    def from_int(val, size=4, is_big_endian=True, signed=False):
        if size not in [2, 4, 8]:
            raise ValueError('size must be 2, 4, or 8, got {}'.format(size))
        # return val.to_bytes(size, byteorder='big' if is_big_endian else 'little', signed=signed)
        fmt = '{}'.format('h' if size == 2 else 'i' if size == 4 else 'q')
        fmt = '{}{}'.format('>' if is_big_endian else '<', fmt if signed else fmt.upper())
        return bytes(struct.pack(fmt, val))
    
    @staticmethod
    def from_fp(val, size=4, is_big_endian=False):
        if size not in [4, 8]:
            raise ValueError('size must be 4, or 8, got {}'.format(size))
        fmt = '{}'.format('f' if size == 4 else 'd')
        fmt = '{}{}'.format('>' if is_big_endian else '<', fmt)
        return bytes(struct.pack(fmt, val))
    
    @staticmethod
    def to_num(data, fmt='>i'):
        return struct.unpack(fmt, bytes(data))[0]
    
    # @staticmethod
    # def to_uint(data, size=4, is_big_endian=True):
    #     if is_big_endian:
    #         return sum([data[i] << (8 * (size - 1 - i)) for i in range(size)])
    #         # return data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
    #     else:
    #         return sum([data[i] << (8 * i) for i in range(size)])
    #         # return data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0]
    
    @staticmethod
    def to_int(data, size=4, is_big_endian=True, signed=False):
        if size not in [2, 4, 8]:
            raise ValueError('size must be 2, 4, or 8, got {}'.format(size))
        if len(data) < size:
            raise ValueError('data length error, data_len={}, size={}'.format(len(data), size))
        # return int.from_bytes(data[:size], byteorder='big' if is_big_endian else 'little', signed=signed)
        fmt = '{}'.format('h' if size == 2 else 'i' if size == 4 else 'q')
        fmt = '{}{}'.format('>' if is_big_endian else '<', fmt if signed else fmt.upper())
        return struct.unpack(fmt, bytes(data[:size]))[0]
    
    @staticmethod
    def to_fp(data, size=4, is_big_endian=False):
        if size not in [4, 8]:
            raise ValueError('size must be 4, or 8, got {}'.format(size))
        if len(data) < size:
            raise ValueError('data length error, data_len={}, size={}'.format(len(data), size))
        fmt = '{}'.format('f' if size == 4 else 'd')
        fmt = '{}{}'.format('>' if is_big_endian else '<', fmt)
        return struct.unpack(fmt, bytes(data[:size]))[0]
    
    ################################ val_to_bytes #################################
    @staticmethod
    def from_u16(val, is_big_endian=True):
        """unsigned short to bytes"""
        # if is_big_endian:
        #     return bytes([val >> 8 & 0xFF, val & 0xFF])
        # else:
        #     return bytes([val & 0xFF, val >> 8 & 0xFF])
        return BytesData.from_int(val, size=2, is_big_endian=is_big_endian)
    
    @staticmethod
    def from_s16(val, is_big_endian=True):
        """signed short to bytes"""
        return BytesData.from_int(val, size=2, is_big_endian=is_big_endian, signed=True)
    
    @staticmethod
    def from_u32(val, is_big_endian=True):
        """unsigned int to bytes"""
        # if is_big_endian:
        #     return bytes([val >> 24 & 0xFF, val >> 16 & 0xFF, val >> 8 & 0xFF, val & 0xFF])
        # else:
        #     return bytes([val & 0xFF, val >> 8 & 0xFF, val >> 16 & 0xFF, val >> 24 & 0xFF])
        return BytesData.from_int(val, size=4, is_big_endian=is_big_endian)
    
    @staticmethod
    def from_s32(val, is_big_endian=True):
        """signed int to bytes"""
        return BytesData.from_int(val, size=4, is_big_endian=is_big_endian, signed=True)
    
    @staticmethod
    def from_u64(val, is_big_endian=True):
        """unsigned long long to bytes"""
        # if is_big_endian:
        #     return bytes([val >> 56 & 0xFF, val >> 48 & 0xFF, val >> 40 & 0xFF, val >> 32 & 0xFF, val >> 24 & 0xFF, val >> 16 & 0xFF, val >> 8 & 0xFF, val & 0xFF])
        # else:
        #     return bytes([val & 0xFF, val >> 8 & 0xFF, val >> 16 & 0xFF, val >> 24 & 0xFF, val >> 32 & 0xFF, val >> 40 & 0xFF, val >> 48 & 0xFF, val >> 56 & 0xFF])
        return BytesData.from_int(val, size=8, is_big_endian=is_big_endian)
    
    @staticmethod
    def from_s64(val, is_big_endian=True):
        """signed long long to bytes"""
        return BytesData.from_int(val, size=8, is_big_endian=is_big_endian, signed=True)

    @staticmethod
    def from_fp32(val, is_big_endian=False):
        """float to bytes"""
        return BytesData.from_fp(val, size=4, is_big_endian=is_big_endian)    
    
    @staticmethod
    def from_fp64(val, is_big_endian=False):
        """double to bytes"""
        return BytesData.from_fp(val, size=8, is_big_endian=is_big_endian)

    @staticmethod
    def from_u16_list(vals, n, is_big_endian=True):
        """unsigned short list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_u16(vals[i], is_big_endian=is_big_endian)
        return data
    
    @staticmethod
    def from_s16_list(vals, n, is_big_endian=True):
        """signed short list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_s16(vals[i], is_big_endian=is_big_endian)
        return data
        
    @staticmethod
    def from_u32_list(vals, n, is_big_endian=True):
        """unsigned int list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_u32(vals[i], is_big_endian=is_big_endian)
        return data
    
    @staticmethod
    def from_s32_list(vals, n, is_big_endian=True):
        """signed int list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_s32(vals[i], is_big_endian=is_big_endian)
        return data
    
    @staticmethod
    def from_u64_list(vals, n, is_big_endian=True):
        """unsigned long long list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_u64(vals[i], is_big_endian=is_big_endian)
        return data
    
    @staticmethod
    def from_s64_list(vals, n, is_big_endian=True):
        """signed long long list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_s64(vals[i], is_big_endian=is_big_endian)
        return data

    @staticmethod
    def from_fp32_list(vals, n, is_big_endian=False):
        """float list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_fp32(vals[i], is_big_endian=is_big_endian)
        return data   
    
    @staticmethod
    def from_fp64_list(vals, n, is_big_endian=False):
        """double list to bytes"""
        data = b''
        for i in range(n):
            data += BytesData.from_fp64(vals[i], is_big_endian=is_big_endian)
        return data

    ################################ bytes_to_val #################################
    @staticmethod
    def to_u16(data, is_big_endian=True):
        """bytes to unsigned short"""
        # if is_big_endian:
        #     return data[0] << 8 | data[1]
        # else:
        #     return data[1] << 8 | data[0]
        return BytesData.to_int(data, size=2, is_big_endian=is_big_endian)
    
    @staticmethod
    def to_s16(data, is_big_endian=True):
        """bytes to signed short"""
        return BytesData.to_int(data, size=2, is_big_endian=is_big_endian, signed=True)
    
    @staticmethod
    def to_u32(data, is_big_endian=True):
        """bytes to unsigned int"""
        # if is_big_endian:
        #     return data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
        # else:
        #     return data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0]
        return BytesData.to_int(data, size=4, is_big_endian=is_big_endian)
    
    @staticmethod
    def to_s32(data, is_big_endian=True):
        """bytes to signed int"""
        return BytesData.to_int(data, size=4, is_big_endian=is_big_endian, signed=True)
    
    @staticmethod
    def to_u64(data, is_big_endian=True):
        """bytes to unsigned long long"""
        # if is_big_endian:
        #     return data[0] << 56 | data[1] << 48 | data[2] << 40 | data[3] << 32 | data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]
        # else:
        #     return data[7] << 56 | data[6] << 48 | data[5] << 40 | data[4] << 32 | data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0]
        return BytesData.to_int(data, size=8, is_big_endian=is_big_endian)
    
    @staticmethod
    def to_s64(data, is_big_endian=True):
        """bytes to signed long long"""
        return BytesData.to_int(data, size=8, is_big_endian=is_big_endian, signed=True)
    
    @staticmethod
    def to_fp32(data, is_big_endian=False):
        """bytes to float"""
        return BytesData.to_fp(data, size=4, is_big_endian=is_big_endian)
    
    @staticmethod
    def to_fp64(data, is_big_endian=False):
        """bytes to double"""
        return BytesData.to_fp(data, size=8, is_big_endian=is_big_endian)

    @staticmethod
    def to_u16_list(data, n, is_big_endian=True):
        """bytes to unsigned short list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_u16(data[i*2:i*2+2], is_big_endian=is_big_endian))
        return vals
    
    @staticmethod
    def to_s16_list(data, n, is_big_endian=True):
        """bytes to signed short list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_s16(data[i*2:i*2+2], is_big_endian=is_big_endian))
        return vals
    
    @staticmethod
    def to_u32_list(data, n, is_big_endian=True):
        """bytes to unsigned int list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_u32(data[i*4:i*4+4], is_big_endian=is_big_endian))
        return vals
    
    @staticmethod
    def to_s32_list(data, n, is_big_endian=True):
        """bytes to signed int list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_s32(data[i*4:i*4+4], is_big_endian=is_big_endian))
        return vals
    
    @staticmethod
    def to_u64_list(data, n, is_big_endian=True):
        """bytes to unsigned long long list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_u64(data[i*8:i*8+8], is_big_endian=is_big_endian))
        return vals
    
    @staticmethod
    def to_s64_list(data, n, is_big_endian=True):
        """bytes to signed long long list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_s64(data[i*8:i*8+8], is_big_endian=is_big_endian))
        return vals
    
    @staticmethod
    def to_fp32_list(data, n, is_big_endian=False):
        """bytes to float list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_fp32(data[i*4:i*4+4], is_big_endian=is_big_endian))
        return vals
    
    @staticmethod
    def to_fp64_list(data, n, is_big_endian=False):
        """bytes to double list"""
        vals = []
        for i in range(n):
            vals.append(BytesData.to_fp64(data[i*8:i*8+8], is_big_endian=is_big_endian))
        return vals
    
    #####################################################################
