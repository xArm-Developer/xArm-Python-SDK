#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Record trajectory
    1. requires firmware 1.2.0 and above support
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


arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)


# Turn on manual mode before recording
arm.set_mode(2)
arm.set_state(0)

arm.start_record_trajectory()

# Analog recording process, here with delay instead
time.sleep(20)

arm.stop_record_trajectory()
arm.save_record_trajectory('test.traj')

time.sleep(1)

# Turn off manual mode after recording
arm.set_mode(0)
arm.set_state(0)
