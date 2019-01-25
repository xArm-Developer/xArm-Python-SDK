#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from .utils import xarm_is_connected


class GPIO(object):
    def __init__(self):
        pass

    @xarm_is_connected(_type='set')
    def set_gpio_addr_16(self, addr, value):
        ret = self.arm_cmd.gpio_addr_w16(addr, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_gpio_addr_16(self, addr):
        ret = self.arm_cmd.gpio_addr_r16(addr)
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_gpio_addr_32(self, addr, value):
        ret = self.arm_cmd.gpio_addr_w32(addr, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_gpio_addr_32(self, addr):
        ret = self.arm_cmd.gpio_addr_r32(addr)
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_gpio_digital(self, ionum=None):
        assert ionum is None or ionum == 1 or ionum == 2, 'The value of parameter ionum can only be 1 or 2 or None.'
        ret = self.arm_cmd.gpio_get_digital()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 28:
                ret[0] = 0
        return ret[0], ret[1:] if ionum is None else ret[ionum]

    @xarm_is_connected(_type='set')
    def set_gpio_digital(self, ionum, value):
        assert ionum == 1 or ionum == 2
        ret = self.arm_cmd.gpio_set_digital(ionum, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_gpio_analog(self, ionum=None):
        if ionum is None:
            ret1 = self.arm_cmd.gpio_get_analog1()
            ret2 = self.arm_cmd.gpio_get_analog2()
            if ret1[0] == 0:
                code = ret2[0]
            else:
                code = ret1[0]
            ret = [code, [ret1[1], ret2[1]]]
        else:
            assert ionum == 1 or ionum == 2, 'The value of parameter ionum can only be 1 or 2 or None.'
            if ionum == 1:
                ret = self.arm_cmd.gpio_get_analog1()
            else:
                ret = self.arm_cmd.gpio_get_analog2()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 28:
                ret[0] = 0
        return ret[0], ret[1]

