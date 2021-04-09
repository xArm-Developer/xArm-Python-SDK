#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: force control with the end force sensor
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

# set parmeters
Kp = 0.005	# 值越小，调整时间越长
Ki = 0.00006  # 值越大，调整时间越小，但超调会变大
Kd = 0.000
xe_max = 1.0
arm.set_force_control_pid([Kp]*6, [Ki]*6, [Kd]*6, [xe_max]*6)

ref_frame = 0        # 1 : base , 0 : tool
force_axis = [0, 0, 1, 0, 0, 0]
force_ref = [0, 0, 2.0 , 0, 0, 0]
arm.config_force_control(ref_frame,  force_axis, force_ref, [0]*6)

arm.ft_sensor_enable(1)
arm.ft_sensor_set_zero()
time.sleep(0.2)

# move robot in force control
arm.ft_sensor_app_set(2)
arm.set_state(0)

time.sleep(5)

arm.ft_sensor_app_set(0)
arm.ft_sensor_enable(0)
arm.disconnect()
