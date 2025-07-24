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
# arm.motion_enable(enable=True)
# arm.clean_error()
# arm.set_mode(0)
# arm.set_state(0)
# time.sleep(0.1)

code, config = arm.get_ft_sensor_config()
if code == 0:
    print('ft_mode: {}'.format(config[0]))
    print('ft_is_started: {}'.format(config[1]))
    print('ft_type: {}'.format(config[2]))
    print('ft_id: {}'.format(config[3]))
    print('ft_freq: {}'.format(config[4]))
    print('ft_mass: {}'.format(config[5]))
    print('ft_dir_bias: {}'.format(config[6]))
    print('ft_centroid: {}'.format(config[7]))
    print('ft_zero: {}'.format(config[8]))
    print('imp_coord: {}'.format(config[9]))
    print('imp_c_axis: {}'.format(config[10]))
    print('M: {}'.format(config[11]))
    print('K: {}'.format(config[12]))
    print('B: {}'.format(config[13]))
    print('f_coord: {}'.format(config[14]))
    print('f_c_axis: {}'.format(config[15]))
    print('f_ref: {}'.format(config[16]))
    print('f_limits: {}'.format(config[17]))
    print('kp: {}'.format(config[18]))
    print('ki: {}'.format(config[19]))
    print('kd: {}'.format(config[20]))
    print('xe_limit: {}'.format(config[21]))

arm.disconnect()
