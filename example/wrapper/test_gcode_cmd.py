#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from xarm.wrapper import XArmAPI

xarm = XArmAPI('192.168.1.185')
xarm.motion_enable(True)
xarm.set_state(0)


xarm.reset()
time.sleep(3)
xarm.send_cmd_sync('G0 X300 Y100 Z200 F50')
time.sleep(2)
xarm.reset()
time.sleep(5)
xarm.disconnect()
