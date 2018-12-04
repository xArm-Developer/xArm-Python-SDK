#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import functools
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI


class XArmDecorator(object):
    def __init__(self, port, *args, **kwargs):
        if port not in self._instances:
            self._arm = XArmAPI(port, *args, **kwargs)
        else:
            instance = self._instances[port]
            if not hasattr(instance, '_arm'):
                instance._arm = XArmAPI(port, *args, **kwargs)
            else:
                self._arm = instance._arm

    def __new__(cls, *args, **kwargs):
        _port = args[0]
        if not hasattr(cls, '_instances'):
            cls._instances = {}
            cls._instances[_port] = super(XArmDecorator, cls).__new__(cls)
        elif _port not in cls._instances:
            cls._instances[_port] = super(XArmDecorator, cls).__new__(cls)
        return cls._instances[_port]

    def __call__(self, func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            func(self._arm, *args, **kwargs)
        return wrapper


@XArmDecorator('192.168.1.113')
def test_arm1_get_position(arm):
    print(arm._arm._port, arm.get_position())


@XArmDecorator('192.168.1.113')
def test_arm1_get_servo_angle(arm):
    print(arm._arm._port, arm.get_servo_angle())


@XArmDecorator('192.168.1.112')
def test_arm2_get_position(arm):
    print(arm._arm._port, arm.get_position())

test_arm1_get_position()
test_arm1_get_servo_angle()
test_arm2_get_position()
test_arm1_get_position()
