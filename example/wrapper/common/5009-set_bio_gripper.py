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

arm.motion_enable(enable=True) 
time.sleep(2)  #Initialize the wait time
arm.set_gripper_enable(enable=True)#gripper enable
time.sleep(5)  #Initialize the wait time
arm.set_mode(0)
arm.set_state(state=0)
arm.set_state(state=0)
#check the version
version_fame = [0x08,0x03,0x08,0x01,0x00,0x03]
version = arm.core.tgpio_set_modbus(version_fame, len(version_fame))
if version[5:11] == [4,4,0,0,0,0]:
    gripper_version = 1
if version[5:11] == [0, 1, 0, 0, 0, 3]:
    gripper_version = 2

gripper_speed = 900  #the speed of gripper

if gripper_version == 1:
    arm.set_gripper_position(-10,wait=False,auto_enable=True,speed=gripper_speed ,timeout=10) #gripper open
    time.sleep(1)
    arm.set_gripper_position(10,wait=False,auto_enable=True,speed=gripper_speed ,timeout=10)  #gripper close
    time.sleep(1)

if gripper_version == 2:
    arm.set_gripper_position(150, wait=False, auto_enable=True, speed=gripper_speed ,timeout=10)  # gripper open
    time.sleep(1)
    arm.set_gripper_position(50, wait=False, auto_enable=True, speed=gripper_speed ,timeout=10)  # gripper close
    time.sleep(1)
