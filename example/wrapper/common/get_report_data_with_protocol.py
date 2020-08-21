#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import sys
import time
import math
import socket
import struct

#######################################################
"""
Example: get report data with socket
"""
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

robot_ip = ip
robot_port = 30002

# 创建socket连接控制盒
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setblocking(True)
sock.settimeout(1)
sock.connect((robot_ip, robot_port))

buffer = sock.recv(4)
while len(buffer) < 4:
    buffer += sock.recv(4 - len(buffer))
size = bytes_to_u32(buffer[:4])

while True:
    buffer += sock.recv(size - len(buffer))
    if len(buffer) < size:
        continue
    data = buffer[:size]
    buffer = buffer[size:]
    print('position:', bytes_to_fp32_list(data[35:59]))

