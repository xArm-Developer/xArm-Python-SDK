#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: standard modbus tcp example
    1. requires firmware 2.1.0 and above support
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


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


arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)


code, ret = arm.read_input_bits(0x00, 16)
print('cgpio_digital_input, code={}, ret={}'.format(code, ret))

code = arm.write_multiple_coil_bits(0x00, [0] * 16)
print('write_multiple_coil_bits, code={}'.format(code))

code, ret = arm.read_coil_bits(0x00, 16)
print('cgpio_digital_output, code={}, ret={}'.format(code, ret))

code = arm.write_multiple_coil_bits(0x00, [1, 0, 1, 0, 1, 1, 0, 1])
print('write_multiple_coil_bits, code={}'.format(code))

code, ret = arm.read_coil_bits(0x00, 16)
print('cgpio_digital_output, code={}, ret={}'.format(code, ret))

code, ret = arm.read_input_registers(0x40, 9, is_signed=True)
ret = ret if code != 0 else list(map(lambda x: x / 10, ret))
print('x/y/z/roll/pitch/yaw/rx/ry/rz, code={}, ret={}'.format(code, ret))
