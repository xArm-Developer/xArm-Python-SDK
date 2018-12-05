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
xarm.motion_enable(enable=True)
xarm.set_state(state=0)

xarm.reset(wait=True)
time.sleep(2)

# set a servo angle: angle value unit is rad, speed unit is rad/s
xarm.set_servo_angle(servo_id=1, angle=3.14, is_radian=True, speed=0.4, wait=True)
# set a servo angle: angle value unit is °, speed unit is °/s
xarm.set_servo_angle(servo_id=2, angle=-45, is_radian=False, speed=20, wait=True)
# set all servo angle: angle value unit is rad, speed unit is rad/s
xarm.set_servo_angle(angle=[-3.14, -30, 10, 0, 0, 0, 0], is_radian=True, speed=0.4, wait=True)
# set all servo angle: angle value unit is °, speed unit is °/s
xarm.set_servo_angle(angle=[45, -45, 0, 0, 0, 0, 0], is_radian=False, speed=20, wait=True)
xarm.reset(wait=True)
time.sleep(2)

xarm.disconnect()
