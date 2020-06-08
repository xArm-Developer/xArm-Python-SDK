#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Hutton <geweipan@ufactory.cc>

"""
Example: Bio Gripper Control
Please make sure that the gripper is attached to the end.
"""
import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from configparser import ConfigParser
parser = ConfigParser()
parser.read('../robot.conf')
try:
    ip = parser.get('xArm', 'ip')
except:
    ip = input('Please input the xArm ip address[192.168.1.194]:')
    if not ip:
        ip = '192.168.1.194'


arm = XArmAPI(ip)
time.sleep(0.5)
if arm.warn_code != 0:
    arm.clean_warn()
if arm.error_code != 0:
    arm.clean_error()

arm.motion_enable(enable=True) #gripper enable
time.sleep(2)  #Initialize the wait time
arm.set_gripper_position(-10,wait=False,auto_enable=True,speed=900,timeout=10) #gripper open
time.sleep(1)
arm.set_gripper_position(10,wait=False,auto_enable=True,speed=900,timeout=10)  #gripper close
