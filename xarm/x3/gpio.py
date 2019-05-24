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

    # @xarm_is_connected(_type='set')
    # def set_tgpio_addr_16(self, addr, value):
    #     ret = self.arm_cmd.tgpio_addr_w16(addr, value)
    #     return ret[0]
    #
    # @xarm_is_connected(_type='get')
    # def get_tgpio_addr_16(self, addr):
    #     ret = self.arm_cmd.tgpio_addr_r16(addr)
    #     return ret[0], ret[1]
    #
    # @xarm_is_connected(_type='set')
    # def set_tgpio_addr_32(self, addr, value):
    #     ret = self.arm_cmd.tgpio_addr_w32(addr, value)
    #     return ret[0]
    #
    # @xarm_is_connected(_type='get')
    # def get_tgpio_addr_32(self, addr):
    #     ret = self.arm_cmd.tgpio_addr_r32(addr)
    #     return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_tgpio_digital(self, ionum=None):
        assert ionum is None or ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1 or None.'
        ret = self.arm_cmd.tgpio_get_digital()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 19:
                ret[0] = 0
        return ret[0], ret[1:] if ionum is None else ret[ionum+1]

    @xarm_is_connected(_type='set')
    def set_tgpio_digital(self, ionum, value):
        assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1.'
        ret = self.arm_cmd.tgpio_set_digital(ionum+1, value)
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 19:
                ret[0] = 0
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
            assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1 or None.'
            if ionum == 0:
                ret = self.arm_cmd.tgpio_get_analog1()
            else:
                ret = self.arm_cmd.tgpio_get_analog2()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 19:
                ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_cgpio_digital(self, ionum=None):
        assert ionum is None or (isinstance(ionum, int) and 7 >= ionum >= 0)
        ret = self.arm_cmd.cgpio_get_auxdigit()
        if ret[0] != 0:
            self.get_err_warn_code()
            if self.error_code != 33:
                ret[0] = 0
        digitals = [0]
        for i in range(8):
            digitals.append(ret[1] >> i & 0x0001)
        return digitals[0], digitals[1:] if ionum is None else digitals[ionum+1]

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
            assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1 or None.'
            if ionum == 0:
                ret = self.arm_cmd.cgpio_get_analog1()
            else:
                ret = self.arm_cmd.cgpio_get_analog2()
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital(self, ionum, value):
        assert isinstance(ionum, int) and 7 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_auxdigit(ionum, value)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_analog(self, ionum, value):
        assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1.'
        if ionum == 1:
            ret = self.arm_cmd.cgpio_set_analog1(value)
        else:
            ret = self.arm_cmd.cgpio_set_analog2(value)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_input_function(self, ionum, fun):
        assert isinstance(ionum, int) and 7 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_infun(ionum, fun)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_output_function(self, ionum, fun):
        assert isinstance(ionum, int) and 7 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_outfun(ionum, fun)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_cgpio_state(self):
        ret = self.arm_cmd.cgpio_get_state()
        # data = {
        #     'state': ret[1],
        #     'error_code': ret[2],
        #     'digital': {
        #         'functional': {
        #             'input': [ret[3] >> i & 0x01 for i in range(8)],
        #             'output': [ret[5] >> i & 0x01 for i in range(8)]
        #         },
        #         'configuring': {
        #             'input': [ret[4] >> i & 0x01 for i in range(8)],
        #             'output': [ret[6] >> i & 0x01 for i in range(8)],
        #         }
        #     },
        #     'analog': {
        #         'input': [ret[7], ret[8]],
        #         'output': [ret[9], ret[10]]
        #     },
        #     'digital_fun': {
        #         'input': ret[11],
        #         'output': ret[12]
        #     }
        # }
        # import json
        # print(json.dumps(data, sort_keys=True, indent=4, skipkeys=True, separators=(',', ':'), ensure_ascii=False))
        # print('cgpio_state:', ret[1])
        # print('cgpio_err_code:', ret[2])
        # print('cgpio_digital_input_fun_state:', bin(ret[3]))
        # print('cgpio_digital_input_cfg_state:', bin(ret[4]))
        # print('cgpio_digital_output_fun_state:', bin(ret[5]))
        # print('cgpio_digital_output_cfg_state:', bin(ret[6]))
        # print('cgpio_analog_1_input:', ret[7])
        # print('cgpio_analog_2_input:', ret[8])
        # print('cgpio_analog_1_output:', ret[9])
        # print('cgpio_analog_2_output:', ret[10])
        # print('cgpio_digital_input_fun:', ret[11])
        # print('cgpio_digital_output_fun:', ret[12])
        return ret[0], ret[1:]

