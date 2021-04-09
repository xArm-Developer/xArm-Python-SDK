#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: teach robot with the end force sensor
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

# set tech parmeters
K_pos = 0               #  x/y/z stiffness coefficient
K_ori = 4               #  Rx/Ry/Rz stiffness coefficient
M = float(0.5)          # mass; unit : kg
c_axis = [0,0,1,0,0,0]  #impendance axis
ref_frame = 1           # 1 : base , 0 : tool

arm.set_impedance_mbk([M, M, M, M*0.01, M*0.01, M*0.01], [K_pos, K_pos, K_pos, K_ori, K_ori, K_ori], [0]*6)
arm.set_impedance_config(ref_frame, c_axis)

arm.ft_sensor_enable(1)
arm.ft_sensor_set_zero()
time.sleep(0.2)

arm.ft_sensor_app_set(1)    #1: impendance control mode
arm.set_state(0)

# You can teach robot now
time.sleep(10)

arm.ft_sensor_app_set(0)
arm.ft_sensor_enable(0)
arm.disconnect()
