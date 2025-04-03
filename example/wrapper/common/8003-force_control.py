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

# set pid parameters for force control
Kp = 0.005  # range: 0 ~ 0.05
Ki = 0.00005 # range: 0 ~ 0.0005
Kd = 0.05  # range: 0 ~ 0.05
linear_v_max = 200.0 # max adjust velocity(mm/s), range: 0 ~ 200
rot_v_max = 0.35 # rad/s range: 0~pi/4
arm.set_force_control_pid([Kp]*6, [Ki]*6, [Kd]*6, [linear_v_max, linear_v_max, linear_v_max, rot_v_max, rot_v_max, rot_v_max])

ref_frame = 1        # 0 : base , 1 : tool
force_axis = [0, 0, 1, 0, 0, 0] # only control force along z axis
# MAKE SURE reference frame and force target sign are correct !!
force_ref = [0, 0, 5.0, 0, 0, 0] # the force(N) that xArm will apply to the environment
code = arm.config_force_control(ref_frame,  force_axis, force_ref, [0]*6) # limits are reserved, just give zeros
if code == 0 and arm.error_code == 0:

    # enable ft sensor communication
    arm.ft_sensor_enable(1)

    # will overwrite previous sensor zero and payload configuration
    arm.ft_sensor_set_zero() # remove this if zero_offset and payload already identified & compensated!
    time.sleep(0.2) # wait for writing zero operation to take effect, do not remove

    # move robot in force control
    arm.ft_sensor_app_set(2)
    # will start after set_state(0)
    arm.set_state(0)

    # keep for 5 secs
    time.sleep(5)

    # remember to reset ft_sensor_app when finished
    arm.ft_sensor_app_set(0)
    arm.ft_sensor_enable(0)
arm.disconnect()
