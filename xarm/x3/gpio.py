#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from .utils import xarm_is_connected, xarm_is_pause, xarm_is_not_simulation_mode, xarm_wait_until_cmdnum_lt_max
from ..core.utils.log import logger
from ..core.config.x_config import XCONF
from .code import APIState
from .base import Base


class GPIO(Base):
    def __init__(self):
        super(GPIO, self).__init__()
        self.cgpio_state = {
            'digital': [-1] * 8,
            'analog': [9999] * 2
        }
        self.tgpio_state = {
            'digital': [-1] * 2,
            'analog': [9999] * 2
        }

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
        # if code != 0:
        #     _, err_warn = self.get_err_warn_code()
        #     if _ in [0, 1, 2]:
        #         if err_warn[0] not in [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 28]:
        #             versions = [ret1[1], ret2[1], ret3[1]]

        return code, '.'.join(map(str, versions))

    @xarm_is_connected(_type='get')
    def get_tgpio_digital(self, ionum=None):
        assert ionum is None or ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1 or None.'
        if self.check_is_simulation_robot():
            return 0, [0, 0] if ionum is None else 0
        ret = self.arm_cmd.tgpio_get_digital()
        if ret[0] == 0:
            self.tgpio_state['digital'] = ret[1:]
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 19:
        #         ret[0] = 0
        return ret[0], ret[1:] if ionum is None else ret[ionum+1]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @xarm_wait_until_cmdnum_lt_max(only_wait=False)
    def set_tgpio_digital(self, ionum, value, delay_sec=0):
        assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1.'
        if delay_sec is not None and delay_sec > 0:
            ret = self.arm_cmd.tgpio_delay_set_digital(ionum, value, delay_sec)
            self.log_api_info('API -> set_tgpio_digital(ionum={}, value={}, delay_sec={}) -> code={}'.format(ionum, value, delay_sec, ret[0]), code=ret[0])
        else:
            ret = self.arm_cmd.tgpio_set_digital(ionum+1, value)
            self.log_api_info('API -> set_tgpio_digital(ionum={}, value={}) -> code={}'.format(ionum, value, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_tgpio_analog(self, ionum=None):
        if self.check_is_simulation_robot():
            return 0, [0, 0] if ionum is None else 0
        if ionum is None:
            ret1 = self.arm_cmd.tgpio_get_analog1()
            ret2 = self.arm_cmd.tgpio_get_analog2()
            if ret1[0] == 0:
                code = ret2[0]
            else:
                code = ret1[0]
            if ret1[0] == 0:
                self.tgpio_state['analog'][0] = ret1[1]
            if ret2[0] == 0:
                self.tgpio_state['analog'][1] = ret2[1]
            ret = [code, [ret1[1], ret2[1]]]
        else:
            assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1 or None.'
            if ionum == 0:
                ret = self.arm_cmd.tgpio_get_analog1()
                if ret[0] == 0:
                    self.tgpio_state['analog'][0] = ret[1]
            else:
                ret = self.arm_cmd.tgpio_get_analog2()
                if ret[0] == 0:
                    self.tgpio_state['analog'][1] = ret[1]
        # if ret[0] != 0:
        #     self.get_err_warn_code()
        #     if self.error_code != 19:
        #         ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_cgpio_digital(self, ionum=None):
        assert ionum is None or (isinstance(ionum, int) and 15 >= ionum >= 0)
        if self.check_is_simulation_robot():
            return 0, [1] * (16 if self._control_box_type_is_1300 else 8) if ionum is None else 1
        ret = self.arm_cmd.cgpio_get_auxdigit()
        digitals = [ret[0]]
        for i in range(16 if self._control_box_type_is_1300 else 8):
            digitals.append(ret[1] >> i & 0x0001)
        return digitals[0], digitals[1:] if ionum is None else digitals[ionum+1]

    @xarm_is_connected(_type='get')
    def get_cgpio_analog(self, ionum=None):
        if self.check_is_simulation_robot():
            return 0, [0, 0] if ionum is None else 0
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
    @xarm_is_not_simulation_mode(ret=0)
    @xarm_wait_until_cmdnum_lt_max(only_wait=False)
    def set_cgpio_digital(self, ionum, value, delay_sec=0):
        assert isinstance(ionum, int) and 15 >= ionum >= 0
        if delay_sec is not None and delay_sec > 0:
            ret = self.arm_cmd.cgpio_delay_set_digital(ionum, value, delay_sec)
            self.log_api_info('API -> set_cgpio_digital(ionum={}, value={}, delay_sec={}) -> code={}'.format(ionum, value, delay_sec, ret[0]), code=ret[0])
        else:
            ret = self.arm_cmd.cgpio_set_auxdigit(ionum, value)
            self.log_api_info('API -> set_cgpio_digital(ionum={}, value={}) -> code={}'.format(ionum, value, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @xarm_wait_until_cmdnum_lt_max(only_wait=False)
    def set_cgpio_analog(self, ionum, value):
        assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1.'
        if ionum == 0:
            ret = self.arm_cmd.cgpio_set_analog1(value)
        else:
            ret = self.arm_cmd.cgpio_set_analog2(value)
        self.log_api_info('API -> set_cgpio_analog(ionum={}, value={}) -> code={}'.format(ionum, value, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_input_function(self, ionum, fun):
        assert isinstance(ionum, int) and 15 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_infun(ionum, fun)
        self.log_api_info('API -> set_cgpio_digital_input_function(ionum={}, fun={}) -> code={}'.format(ionum, fun, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_cgpio_digital_output_function(self, ionum, fun):
        assert isinstance(ionum, int) and 15 >= ionum >= 0
        ret = self.arm_cmd.cgpio_set_outfun(ionum, fun)
        self.log_api_info('API -> set_cgpio_digital_output_function(ionum={}, fun={}) -> code={}'.format(ionum, fun, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_cgpio_state(self):
        ret = self.arm_cmd.cgpio_get_state()
        code, states = ret[0], ret[1:]
        if not self._control_box_type_is_1300:
            states[-1] = states[-1][:8]
            states[-2] = states[-2][:8]
        if code == 0 and states[0] == 0 and states[1] == 0:
            self.cgpio_state['digital'] = [states[3] >> i & 0x0001 if states[10][i] in [0, 255] else 1 for i in range(len(states[10]))]
            self.cgpio_state['analog'] = [states[6], states[7]]
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
        return code, states

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @xarm_wait_until_cmdnum_lt_max(only_wait=False)
    def set_suction_cup(self, on, wait=True, timeout=3, delay_sec=None):
        if on:
            code1 = self.set_tgpio_digital(ionum=0, value=1, delay_sec=delay_sec)
            code2 = self.set_tgpio_digital(ionum=1, value=0, delay_sec=delay_sec)
        else:
            code1 = self.set_tgpio_digital(ionum=0, value=0, delay_sec=delay_sec)
            code2 = self.set_tgpio_digital(ionum=1, value=1, delay_sec=delay_sec)
        code = code1 if code2 == 0 else code2
        if code == 0 and wait:
            start = time.time()
            code = APIState.SUCTION_CUP_TOUT
            if delay_sec is not None and delay_sec > 0:
                timeout += delay_sec
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
                if not self.connected or self.state == 4:
                    code = APIState.EMERGENCY_STOP
                    break
                time.sleep(0.1)
        self.log_api_info('API -> set_suction_cup(on={}, wait={}, delay_sec={}) -> code={}'.format(on, wait, delay_sec, code), code=code)
        return code

    @xarm_is_connected(_type='get')
    def get_suction_cup(self):
        return self.get_tgpio_digital(ionum=0)

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @xarm_wait_until_cmdnum_lt_max(only_wait=False)
    def set_tgpio_digital_with_xyz(self, ionum, value, xyz, fault_tolerance_radius):
        assert isinstance(ionum, int) and 1 >= ionum >= 0
        assert fault_tolerance_radius >= 0, 'The value of parameter fault_tolerance_radius must be greater than or equal to 0.'
        ret = self.arm_cmd.tgpio_position_set_digital(ionum, value, xyz, fault_tolerance_radius)
        self.log_api_info('API -> set_tgpio_digital_with_xyz(ionum={}, value={}, xyz={}, fault_tolerance_radius={}) -> code={}'.format(ionum, value, xyz, fault_tolerance_radius, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @xarm_wait_until_cmdnum_lt_max(only_wait=False)
    def set_cgpio_digital_with_xyz(self, ionum, value, xyz, fault_tolerance_radius):
        assert isinstance(ionum, int) and 15 >= ionum >= 0
        assert fault_tolerance_radius >= 0, 'The value of parameter fault_tolerance_radius must be greater than or equal to 0.'
        ret = self.arm_cmd.cgpio_position_set_digital(ionum, value, xyz, fault_tolerance_radius)
        self.log_api_info('API -> set_cgpio_digital_with_xyz(ionum={}, value={}, xyz={}, fault_tolerance_radius={}) -> code={}'.format(ionum, value, xyz, fault_tolerance_radius, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @xarm_wait_until_cmdnum_lt_max(only_wait=False)
    def set_cgpio_analog_with_xyz(self, ionum, value, xyz, fault_tolerance_radius):
        assert ionum == 0 or ionum == 1, 'The value of parameter ionum can only be 0 or 1.'
        assert fault_tolerance_radius >= 0, 'The value of parameter fault_tolerance_radius must be greater than or equal to 0.'
        ret = self.arm_cmd.cgpio_position_set_analog(ionum, value, xyz, fault_tolerance_radius)
        self.log_api_info('API -> set_cgpio_analog_with_xyz(ionum={}, value={}, xyz={}, fault_tolerance_radius={}) -> code={}'.format(ionum, value, xyz, fault_tolerance_radius, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def config_io_reset_when_stop(self, io_type, on_off):
        ret = self.arm_cmd.config_io_stop_reset(io_type, int(on_off))
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=False)
    def check_air_pump_state(self, state, timeout=3):
        start_time = time.time()
        is_first = True
        while is_first or time.time() - start_time < timeout:
            is_first = False
            if not self.connected or self.state == 4:
                return False
            ret = self.get_suction_cup()
            if ret[0] == XCONF.UxbusState.ERR_CODE:
                return False
            if ret[0] == 0:
                if state and ret[1] == 1:
                    return True
                if not state and ret[1] == 0:
                    return True
            time.sleep(0.1)
        return False


