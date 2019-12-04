#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from .utils import xarm_is_connected, xarm_is_pause
from ..core.utils.log import logger
from ..core.config.x_config import XCONF
from .code import APIState


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
    def get_tgpio_version(self):
        versions = ['*', '*', '*']
        ret1 = self.arm_cmd.tgpio_addr_r16(0x0801)
        ret2 = self.arm_cmd.tgpio_addr_r16(0x0802)
        ret3 = self.arm_cmd.tgpio_addr_r16(0x0803)

        code = 0

        if ret1[0] == 0 and len(ret1) == 2:
            versions[0] = ret1[1]
        else:
            code = ret1[0]

        if ret2[0] == 0 and len(ret2) == 2:
            versions[1] = ret2[1]
        else:
            code = ret2[0]

        if ret3[0] == 0 and len(ret3) == 2:
            versions[2] = ret3[1]
        else:
            code = ret3[0]

        return code, '.'.join(map(str, versions))

    @xarm_is_connected(_type='get')
    def get_tgpio_digital(self, ionum=None):
        assert ionum is None or ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1 or None.'
        ret = self.arm_cmd.tgpio_get_digital()
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 19:
        #         ret[0] = 0
        return ret[0], ret[1:] if ionum is None else ret[ionum+1]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_tgpio_digital(self, ionum, value):
        assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1.'
        ret = self.arm_cmd.tgpio_set_digital(ionum+1, value)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 19 and self.error_code != 28:
        #         ret[0] = 0
        logger.info('API -> set_tgpio_digital -> ret={}, io={}, value={}'.format(ret[0], ionum, value))
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
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 19:
        #         ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_cgpio_digital(self, ionum=None):
        assert ionum is None or (isinstance(ionum, int) and 7 >= ionum >= 0)
        ret = self.arm_cmd.cgpio_get_auxdigit()
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        digitals = [ret[0]]
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
    @xarm_is_pause(_type='set')
    def set_cgpio_digital(self, ionum, value):
        assert isinstance(ionum, int) and 7 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_auxdigit(ionum, value)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        logger.info('API -> set_cgpio_digital -> ret={}, io={}, value={}'.format(ret[0], ionum, value))
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_cgpio_analog(self, ionum, value):
        assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1.'
        if ionum == 0:
            ret = self.arm_cmd.cgpio_set_analog1(value)
        else:
            ret = self.arm_cmd.cgpio_set_analog2(value)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        logger.info('API -> set_cgpio_analog -> ret={}, io={}, value={}'.format(ret[0], ionum, value))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_input_function(self, ionum, fun):
        assert isinstance(ionum, int) and 7 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_infun(ionum, fun)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        logger.info('API -> set_cgpio_infun -> ret={}, io={}, fun={}'.format(ret[0], ionum, fun))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_output_function(self, ionum, fun):
        assert isinstance(ionum, int) and 7 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_outfun(ionum, fun)
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 33:
        #         ret[0] = 0
        logger.info('API -> set_cgpio_outfun -> ret={}, io={}, fun={}'.format(ret[0], ionum, fun))
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
        # # print(json.dumps(data, sort_keys=True, indent=4, skipkeys=True, separators=(',', ':'), ensure_ascii=False))
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

    @xarm_is_connected(_type='set')
    def set_suction_cup(self, on, wait=True, timeout=3):
        if on:
            code1 = self.set_tgpio_digital(ionum=0, value=1)
            code2 = self.set_tgpio_digital(ionum=1, value=0)
        else:
            code1 = self.set_tgpio_digital(ionum=0, value=0)
            code2 = self.set_tgpio_digital(ionum=1, value=1)
        code = code1 if code2 == 0 else code2
        if code == 0 and wait:
            start = time.time()
            code = APIState.SUCTION_CUP_TOUT
            while time.time() - start < timeout:
                ret = self.get_suction_cup()
                if ret[0] == XCONF.UxbusState.ERR_CODE:
                    code = XCONF.UxbusState.ERR_CODE
                    break
                if ret[0] == 0:
                    if on and ret[1] == 1:
                        code = 0
                        break
                    if not on and ret[1] == 0:
                        code = 0
                        break
                time.sleep(0.1)
        logger.info('API -> set_suction_cup -> ret={}, on={}, wait={}'.format(code, on, wait))
        return code

    @xarm_is_connected(_type='get')
    def get_suction_cup(self):
        return self.get_tgpio_digital(ionum=0)
