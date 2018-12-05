#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from xarm.wrapper import XArmAPI

xarm = XArmAPI(port='192.168.1.113',
               enable_heartbeat=True,
               enable_report=True,
               report_type='normal')
xarm.motion_enable(enable=True)
xarm.set_state(state=0)


xarm.reset(wait=True)
time.sleep(3)
ret = xarm.send_cmd_sync('G1 X300 Y0 Z150 F50')
time.sleep(2)
xarm.reset(wait=True)
time.sleep(5)
xarm.disconnect()
