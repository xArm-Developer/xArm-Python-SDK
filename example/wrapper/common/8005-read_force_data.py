#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: read force raw data and external data
"""

import sys
import time
import math
import socket
import struct
from xarm.wrapper import XArmAPI
#######################################################

if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


def bytes_to_fp32(bytes_data, is_big_endian=False):
    return struct.unpack('>f' if is_big_endian else '<f', bytes_data)[0]

def bytes_to_fp32_list(bytes_data, n=0, is_big_endian=False):
    ret = []
    count = n if n > 0 else len(bytes_data) // 4
    for i in range(count):
        ret.append(bytes_to_fp32(bytes_data[i * 4: i * 4 + 4], is_big_endian))
    return ret

def bytes_to_u32(data):
    data_u32 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
    return data_u32

arm = XArmAPI(ip, enable_report=True)
arm.motion_enable(enable=True)
arm.ft_sensor_enable(0)

arm.clean_error()
arm.clean_warn()
arm.ft_sensor_enable(1)
time.sleep(0.5)
arm.ft_sensor_set_zero()

# socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setblocking(True)
sock.settimeout(1)
sock.connect((ip, 30003))

buffer = sock.recv(4)
while len(buffer) < 4:
    buffer += sock.recv(4 - len(buffer))
size = bytes_to_u32(buffer[:4])

start = time.time()
error_code = 0
while (error_code == 0):
    buffer += sock.recv(size - len(buffer))
    if len(buffer) < size:
        continue
    data = buffer[:size]
    buffer = buffer[size:]
    if time.time() - start > 1:
        ret, ret1 = arm.get_err_warn_code()
        if ret1[0] == 0:
            print(time.strftime("%Y-%m-%d %H:%M:%S  ", time.localtime()), end = '')
            print('position:', bytes_to_fp32_list(data[35:59]))
            print('exe_force:', bytes_to_fp32_list(data[87:111]))
            print('raw_force:', bytes_to_fp32_list(data[111:135]))
            start = time.time()
        else:
            print(arm.get_err_warn_code())
            error_code = 1

arm.ft_sensor_enable(0)
sock.close()