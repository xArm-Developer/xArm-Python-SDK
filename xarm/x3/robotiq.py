#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from ..core.utils.log import logger
from .code import APIState
from .base import Base
from .decorator import xarm_is_connected, xarm_is_not_simulation_mode


class RobotIQ(Base):
    def __init__(self):
        super(RobotIQ, self).__init__()
        self.__robotiq_openmm = None
        self.__robotiq_closemm = None
        self.__robotiq_aCoef = None
        self.__robotiq_bCoef = None
        self._robotiq_status = {
            'gOBJ': 0,  # 1和2表示停止并抓取到物体，3表示停止但没有抓取到物体，0表示正在动也没有抓取到物体
            'gSTA': 0,  # 3表示激活完成
            'gGTO': 0,
            'gACT': 0,
            'kFLT': 0,
            'gFLT': 0,  # 错误码
            'gPR': 0,  # 抓取器的目标位置
            'gPO': 0,  # 通过编码器获取抓取器的实际位置, 值介于0x00和0xFF之间, 0xFF为全闭合, 0x00为全张开
            'gCU': 0,  # 从电机驱动器上瞬时读电流, 值介于0x00和0xFF之间, 等效电流大约为=10 * value mA
        }

    @property
    def robotiq_error_code(self):
        return self.robotiq_status['gFLT']

    @property
    def robotiq_status(self):
        return self._robotiq_status

    def __robotiq_send_modbus(self, data_frame, min_res_len=0):
        code = self.checkset_modbus_baud(self._default_robotiq_baud)
        if code != 0:
            return code, []
        return self.getset_tgpio_modbus_data(data_frame, min_res_len=min_res_len, ignore_log=True)

    @xarm_is_connected(_type='get')
    def __robotiq_set(self, params):
        data_frame = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, len(params)]
        data_frame.extend(params)
        return self.__robotiq_send_modbus(data_frame, 6)

    @xarm_is_connected(_type='get')
    def __robotiq_get(self, params):
        data_frame = [0x09, 0x03]
        data_frame.extend(params)
        code, ret = self.__robotiq_send_modbus(data_frame, 3 + 2 * params[-1])
        if code == 0 and len(ret) >= 5:
            gripper_status_reg = ret[3]
            self._robotiq_status['gOBJ'] = (gripper_status_reg & 0xC0) >> 6
            self._robotiq_status['gSTA'] = (gripper_status_reg & 0x30) >> 4
            self._robotiq_status['gGTO'] = (gripper_status_reg & 0x08) >> 3
            self._robotiq_status['gACT'] = gripper_status_reg & 0x01
            if len(ret) >= 7:
                fault_status_reg = ret[5]
                self._robotiq_status['kFLT'] = (fault_status_reg & 0xF0) >> 4
                self._robotiq_status['gFLT'] = fault_status_reg & 0x0F
                self._robotiq_status['gPR'] = ret[6]
            if len(ret) >= 9:
                self._robotiq_status['gPO'] = ret[7]
                self._robotiq_status['gCU'] = ret[8]
            if self._robotiq_status['gSTA'] == 3 and (self._robotiq_status['gFLT'] == 0 or self._robotiq_status['gFLT'] == 9):
                self.robotiq_is_activated = True
            else:
                self.robotiq_is_activated = False
        return code, ret

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def robotiq_reset(self):
        params = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        code, ret = self.__robotiq_set(params)
        self.log_api_info('API -> robotiq_reset -> code={}, response={}'.format(code, ret), code=code)
        return code, ret

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def robotiq_set_activate(self, wait=True, timeout=3):
        params = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        code, ret = self.__robotiq_set(params)
        if wait and code == 0:
            code = self.robotiq_wait_activation_completed(timeout)
        self.log_api_info('API -> robotiq_set_activate ->code={}, response={}'.format(code, ret), code=code)
        if code == 0:
            self.robotiq_is_activated = True
        return code, ret

    @xarm_is_connected(_type='get')
    def robotiq_set_position(self, pos, speed=0xFF, force=0xFF, wait=True, timeout=5, **kwargs):
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move()
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code, 0
        if self.check_is_simulation_robot():
            return 0, 0
        if kwargs.get('auto_enable') and not self.robotiq_is_activated:
            self.robotiq_reset()
            self.robotiq_set_activate(wait=True)
        params = [0x09, 0x00, 0x00, pos, speed, force]
        code, ret = self.__robotiq_set(params)
        if wait and code == 0:
            code = self.robotiq_wait_motion_completed(timeout, **kwargs)
        self.log_api_info('API -> robotiq_set_position ->code={}, response={}'.format(code, ret), code=code)
        return code, ret

    def robotiq_open(self, speed=0xFF, force=0xFF, wait=True, timeout=5, **kwargs):
        return self.robotiq_set_position(0x00, speed=speed, force=force, wait=wait, timeout=timeout, **kwargs)

    def robotiq_close(self, speed=0xFF, force=0xFF, wait=True, timeout=5, **kwargs):
        return self.robotiq_set_position(0xFF, speed=speed, force=force, wait=wait, timeout=timeout, **kwargs)

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def robotiq_get_status(self, number_of_registers=3):
        number_of_registers = 3 if number_of_registers not in [1, 2, 3] else number_of_registers
        params = [0x07, 0xD0, 0x00, number_of_registers]
        # params = [0x07, 0xD0, 0x00, 0x01]
        # params = [0x07, 0xD0, 0x00, 0x03]
        return self.__robotiq_get(params)

    def robotiq_wait_activation_completed(self, timeout=3):
        failed_cnt = 0
        expired = time.monotonic() + timeout if timeout is not None and timeout > 0 else 0
        code = APIState.WAIT_FINISH_TIMEOUT
        while expired == 0 or time.monotonic() < expired:
            _, ret = self.robotiq_get_status(number_of_registers=3)
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                gFLT = self._robotiq_status['gFLT']
                gSTA = self._robotiq_status['gSTA']
                code = APIState.END_EFFECTOR_HAS_FAULT if gFLT != 0 and not (gFLT == 5 and gSTA == 1) \
                    else 0 if gSTA == 3 else code
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.05)
        return code

    def robotiq_wait_motion_completed(self, timeout=5, **kwargs):
        failed_cnt = 0
        expired = time.monotonic() + timeout if timeout is not None and timeout > 0 else 0
        code = APIState.WAIT_FINISH_TIMEOUT
        check_detected = kwargs.get('check_detected', False)
        while expired == 0 or time.monotonic() < expired:
            _, ret = self.robotiq_get_status(number_of_registers=3)
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                gFLT = self._robotiq_status['gFLT']
                gSTA = self._robotiq_status['gSTA']
                gOBJ = self._robotiq_status['gOBJ']
                code = APIState.END_EFFECTOR_HAS_FAULT if gFLT != 0 and not (gFLT == 5 and gSTA == 1) \
                    else 0 if (check_detected and (gOBJ == 1 or gOBJ == 2)) or (gOBJ == 1 or gOBJ == 2 or gOBJ == 3) \
                    else code
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.05)
        if self.robotiq_error_code != 0:
            print('ROBOTIQ Gripper ErrorCode: {}'.format(self.robotiq_error_code))
        if code == 0 and not self.robotiq_is_activated:
            code = APIState.END_EFFECTOR_NOT_ENABLED
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=False)
    def check_robotiq_is_catch(self, timeout=5):
        return self.robotiq_wait_motion_completed(timeout=timeout, check_detected=True) == 0

    def robotiq_calibrate_mm(self, closemm, openmm):
        ret = self.robotiq_open(wait=True)
        if ret[0] == 0:
            open_bit = self._robotiq_status['gPO']
            ret = self.robotiq_close(wait=True)
            if ret[0] == 0:
                close_bit = self._robotiq_status['gPO']
                self.__robotiq_aCoef = (closemm - openmm) / (close_bit - open_bit)
                self.__robotiq_bCoef = (openmm * close_bit - closemm * open_bit) / (close_bit - open_bit)
                self.__robotiq_closemm = closemm
                self.__robotiq_openmm = openmm
                return 0
        return ret[0]

    def robotiq_set_position_mm(self, pos_mm, speed=0xFF, force=0xFF, wait=False, timeout=5, check_detected=False):
        if self.__robotiq_openmm is None or self.__robotiq_closemm is None:
            print('You have to calibrate the gripper before using the function robotiq_set_position_mm()')
        elif pos_mm > self.__robotiq_openmm:
            print("The maximum opening is {}".format(self.__robotiq_openmm))
        else:
            pos = int(self.__robotiq_mm_to_bit(pos_mm))
            return self.robotiq_set_position(pos, speed=speed, force=force, wait=wait, timeout=timeout, check_detected=check_detected)

    def __robotiq_mm_to_bit(self, mm):
        if self.__robotiq_aCoef is None or self.__robotiq_bCoef is None:
            print('You have to calibrate the gripper before using the function robotiq_mm_to_bit()')
        else:
            return (mm - self.__robotiq_bCoef) / self.__robotiq_aCoef

    def __robotiq_bit_to_mm(self, bit):
        if self.__robotiq_aCoef is None or self.__robotiq_bCoef is None:
            print('You have to calibrate the gripper before using the function robotiq_bit_to_mm()')
        else:
            return self.__robotiq_aCoef * bit + self.__robotiq_bCoef

