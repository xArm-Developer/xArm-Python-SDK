#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import time
import functools
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI(port='192.168.1.145')

arm.run_blockly_app('./example.xml')
# source_path = 'C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm7\\app\\myapp\\pour_water\\app.xml'
# arm.run_blockly_app(source_path)

time.sleep(1)
arm.disconnect()
