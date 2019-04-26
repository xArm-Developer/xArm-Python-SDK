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
    def set_tgpio_addr_16(self, addr, value):
        ret = self.arm_cmd.tgpio_addr_w16(addr, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_tgpio_addr_16(self, addr):
        ret = self.arm_cmd.tgpio_addr_r16(addr)
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_tgpio_addr_32(self, addr, value):
        ret = self.arm_cmd.tgpio_addr_w32(addr, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_tgpio_addr_32(self, addr):
        ret = self.arm_cmd.tgpio_addr_r32(addr)
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_tgpio_digital(self, ionum=None):
        assert ionum is None or ionum == 1 or ionum == 2, 'The value of parameter ionum can only be 1 or 2 or None.'
        ret = self.arm_cmd.tgpio_get_digital()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 28:
                ret[0] = 0
        return ret[0], ret[1:] if ionum is None else ret[ionum]

    @xarm_is_connected(_type='set')
    def set_tgpio_digital(self, ionum, value):
        assert ionum == 1 or ionum == 2
        ret = self.arm_cmd.tgpio_set_digital(ionum, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_tgpio_analog(self, ionum=None):
        if ionum is None:
            ret1 = self.arm_cmd.tgpio_get_analog1()
            ret2 = self.arm_cmd.tgpio_get_analog2()
            if ret1[0] == 0:
                code = ret2[0]
            else:
                code = ret1[0]
            ret = [code, [ret1[1], ret2[1]]]
        else:
            assert ionum == 1 or ionum == 2, 'The value of parameter ionum can only be 1 or 2 or None.'
            if ionum == 1:
                ret = self.arm_cmd.tgpio_get_analog1()
            else:
                ret = self.arm_cmd.tgpio_get_analog2()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 28:
                ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_cgpio_digital(self, ionum=None):
        assert ionum is None or (6 >= ionum >= 1)
        ret = self.arm_cmd.cgpio_get_auxdigit()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 33:
                ret[0] = 0
        digitals = [0]
        for i in range(6):
            digitals.append(ret[1] >> i & 0x0001)
        return digitals[0], digitals[1:] if ionum is None else digitals[ionum]

    @xarm_is_connected(_type='get')
    def get_cgpio_analog(self, ionum=None):
        if ionum is None:
            ret1 = self.arm_cmd.cgpio_get_analog1()
            ret2 = self.arm_cmd.cgpio_get_analog2()
            if ret1[0] == 0:
                code = ret2[0]
            else:
                code = ret1[0]
            ret = [code, [ret1[1], ret2[1]]]
        else:
            assert ionum == 1 or ionum == 2, 'The value of parameter ionum can only be 1 or 2 or None.'
            if ionum == 1:
                ret = self.arm_cmd.cgpio_get_analog1()
            else:
                ret = self.arm_cmd.cgpio_get_analog2()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 33:
                ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital(self, ionum, value):
        assert 6 >= ionum >= 1
        ret = self.arm_cmd.cgpio_set_auxdigit(ionum-1, value)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_analog(self, ionum, value):
        assert ionum == 1 or ionum == 2, 'The value of parameter ionum can only be 1 or 2 or None.'
        if ionum == 1:
            ret = self.arm_cmd.cgpio_set_analog1(value)
        else:
            ret = self.arm_cmd.cgpio_set_analog2(value)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_input_function(self, ionum, fun):
        assert 6 >= ionum >= 1
        ret = self.arm_cmd.cgpio_set_infun(ionum-1, fun)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_output_function(self, ionum, fun):
        assert 6 >= ionum >= 1
        ret = self.arm_cmd.cgpio_set_outfun(ionum - 1, fun)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_cgpio_state(self):
        ret = self.arm_cmd.cgpio_get_state()
        return ret[0], ret[1:]

