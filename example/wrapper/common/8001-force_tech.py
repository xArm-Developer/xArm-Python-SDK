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

# Hand guided Cartesian jogging can be achieved by setting zero stiffness to target axis
# set teach parameters
K_pos = 0               #  x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
K_ori = 0               #  Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)

# Attention: for M and J, smaller value means less effort to drive the arm, but may also be less stable, please be careful. 
M = float(0.05)         #  x/y/z equivalent mass; range: 0.02 ~ 1 kg
J = M * 0.01     #  Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)

c_axis = [0,0,1,0,0,0]  # compliant axis: z
ref_frame = 0           # 0 : base , 1 : tool

arm.set_ft_sensor_admittance_parameters([M, M, M, J, J, J], [K_pos, K_pos, K_pos, K_ori, K_ori, K_ori], [0]*6) # B(damping) is reserved, give zeros
arm.set_ft_sensor_admittance_parameters(ref_frame, c_axis)

# enable ft sensor communication
arm.set_ft_sensor_enable(1)
# will overwrite previous sensor zero and payload configuration
# arm.set_ft_sensor_zero() # remove this if zero_offset and payload already identified & compensated!
time.sleep(0.2) # wait for writing zero operation to take effect, do not remove

arm.set_ft_sensor_mode(1)    #1: admittance control mode
arm.set_state(0) # will start after set_state(0)

# You can drag and teach robot along compliant axis now
time.sleep(10)

# remember to reset ft_sensor_app when finished
arm.set_ft_sensor_mode(0)
arm.set_ft_sensor_enable(0)
arm.disconnect()
