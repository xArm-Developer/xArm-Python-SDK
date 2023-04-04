#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: read report data example
    1. requires firmware 2.1.0 and above support
"""

import os
import sys
import time
import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.core.comm import SocketPort
from xarm.core.utils import convert


#######################################################
"""
Just for test example
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


sock = SocketPort(ip, 30001)

while sock.connected:
    data = sock.read(timeout=1)
    if data == -1:
        time.sleep(0.1)
        continue
    total = convert.bytes_to_u32(data[0:4])
    angles = convert.bytes_to_fp32s(data[7:35], 7)
    poses = convert.bytes_to_fp32s(data[35:59], 6)
    print('total={}, now={}'.format(total, datetime.datetime.now()))
    print('angles: {}'.format(angles))
    print('poses: {}'.format(poses))

