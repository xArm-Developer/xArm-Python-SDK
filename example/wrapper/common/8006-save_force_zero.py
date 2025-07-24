#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: save force sensor zero data to xArm controller.
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

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

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)

arm.set_ft_sensor_enable(1)
time.sleep(0.1)
arm.set_ft_sensor_zero()
time.sleep(0.2)

#If you save the force sensor zero offset, you don't need to call ft_sensor_set_zero() every time you start
arm.save_conf()
arm.disconnect()
