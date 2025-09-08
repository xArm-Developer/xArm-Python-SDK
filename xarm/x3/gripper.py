#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import math
import time
import struct
from ..core.config.x_config import XCONF
from ..core.utils.log import logger
from ..core.utils import convert
from .code import APIState
from .gpio import GPIO
from .decorator import xarm_is_connected, xarm_wait_until_not_pause, xarm_is_not_simulation_mode
from decimal import Decimal, ROUND_HALF_UP

# 各自由度角度、力控、速度寄存器地址
FINGER_ADDR = {
    1: 0x05CE,  # 小拇指
    2: 0x05D0,  # 无名指
    3: 0x05D2,  # 中指
    4: 0x05D4,  # 食指
    5: 0x05D6,  # 大拇指弯曲
    6: 0x05D8,  # 大拇指旋转
}
GET_POS_ADDR = {
    1: 0x060A,
    2: 0x060C,
    3: 0x060E,
    4: 0x0610,
    5: 0x0612,
    6: 0x0614
}
FINGER_STATUS_ADDR = {
    1: 0x064C,
    2: 0x064D,
    3: 0x064E,
    4: 0x064F,
    5: 0x0650,
    6: 0x0651
}
FORCE_ADDR = {
    1: 0x05DA,  # 小拇指
    2: 0x05DC,
    3: 0x05DE,
    4: 0x05E0,
    5: 0x05E2,
    6: 0x05E4,
}
SPEED_ADDR = {
    1: 0x05F2,  # 小拇指
    2: 0x05F4,
    3: 0x05F6,
    4: 0x05F8,
    5: 0x05FA,
    6: 0x05FC,
}

HAND_ID = 0x01

class Gripper(GPIO):
    def __init__(self):
        super(Gripper, self).__init__()
        self._gripper_error_code = 0
        self._bio_gripper_version = 0  # 0表示未曾获取SN判别，1表示旧的(获取SN失败), 2表示新的(获取SN成功)
        self._bio_gripper_mode = -1    # -1表示未获取

    @property
    def gripper_error_code(self):
        return self._gripper_error_code

    @gripper_error_code.setter
    def gripper_error_code(self, val):
        self._gripper_error_code = val

    @property
    def gripper_is_support_status(self):
        if self.gripper_version_numbers[0] == -1 or self.gripper_version_numbers[1] == -1 or self.gripper_version_numbers[2] == -1:
            self.get_gripper_version()
        return self.gripper_version_numbers[0] > 3 \
               or (self.gripper_version_numbers[0] == 3 and self.gripper_version_numbers[1] > 4) \
               or (self.gripper_version_numbers[0] == 3 and self.gripper_version_numbers[1] == 4 and self.gripper_version_numbers[2] >= 3)

    @xarm_is_connected(_type='get')
    def get_gripper_status(self):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code, 0
        ret = self.arm_cmd.gripper_modbus_r16s(0x0000, 1)
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        status = 0
        if ret[0] == 0 and len(ret) == 7:
            status = convert.bytes_to_u16(ret[5:7])
        return ret[0], status

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, '*.*.*'))
    def get_gripper_version(self):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code, '*.*.*'
        versions = ['*', '*', '*']
        ret1 = self.arm_cmd.gripper_modbus_r16s(0x0801, 1)
        ret2 = self.arm_cmd.gripper_modbus_r16s(0x0802, 1)
        ret3 = self.arm_cmd.gripper_modbus_r16s(0x0803, 1)
        ret1[0] = self._check_modbus_code(ret1, only_check_code=True)
        ret2[0] = self._check_modbus_code(ret2, only_check_code=True)
        ret3[0] = self._check_modbus_code(ret3, only_check_code=True)

        code = 0

        if ret1[0] == 0 and len(ret1) == 7:
            versions[0] = convert.bytes_to_u16(ret1[5:7])
            self.gripper_version_numbers[0] = versions[0]
        else:
            code = ret1[0]

        if ret2[0] == 0 and len(ret2) == 7:
            versions[1] = convert.bytes_to_u16(ret2[5:7])
            self.gripper_version_numbers[1] = versions[1]
        else:
            code = ret2[0]

        if ret3[0] == 0 and len(ret3) == 7:
            versions[2] = convert.bytes_to_u16(ret3[5:7])
            self.gripper_version_numbers[2] = versions[2]
        else:
            code = ret3[0]

        return code, '.'.join(map(str, versions))

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_gripper_enable(self, enable, is_modbus=True):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if is_modbus:
            return self._set_modbus_gripper_enable(enable)
        else:
            return self._set_gripper_enable(enable)

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_gripper_mode(self, mode, is_modbus=True):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if is_modbus:
            return self._set_modbus_gripper_mode(mode)
        else:
            return self._set_gripper_mode(mode)

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_gripper_speed(self, speed, is_modbus=True):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if is_modbus:
            return self._set_modbus_gripper_speed(speed)
        else:
            return self._set_gripper_speed(speed)

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_gripper_position(self, is_modbus=True):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code, None
        if is_modbus:
            return self._get_modbus_gripper_position()
        else:
            return self._get_gripper_position()

    @xarm_wait_until_not_pause
    @xarm_is_connected(_type='set')
    def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, is_modbus=True, **kwargs):
        if is_modbus:
            return self._set_modbus_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout, **kwargs)
        else:
            return self._set_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout, **kwargs)

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_gripper_err_code(self, is_modbus=True):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code, 0
        if is_modbus:
            return self._get_modbus_gripper_err_code()
        else:
            return self._get_gripper_err_code()

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def clean_gripper_error(self, is_modbus=True):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if is_modbus:
            return self._clean_modbus_gripper_error()
        else:
            return self._clean_gripper_error()

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_gripper_zero(self, is_modbus=True):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if is_modbus:
            return self._set_modbus_gripper_zero()
        else:
            return self._set_gripper_zero()

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_gripper_status(self, status, delay_sec=0, sync=True):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if status:
            code1 = self.set_tgpio_digital(ionum=0, value=1, delay_sec=delay_sec, sync=sync)
            code2 = self.set_tgpio_digital(ionum=1, value=0, delay_sec=delay_sec, sync=sync)
        else:
            code1 = self.set_tgpio_digital(ionum=0, value=0, delay_sec=delay_sec, sync=sync)
            code2 = self.set_tgpio_digital(ionum=1, value=1, delay_sec=delay_sec, sync=sync)
        code = code1 if code2 == 0 else code2
        self.log_api_info('API -> set_gripper_status(status={}, delay_sec={}) -> code={}'.format(status, delay_sec, code), code=code)
        return code

    ########################### Old Protocol #################################
    @xarm_is_connected(_type='set')
    def _set_gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_set_en(int(enable))
        self.log_api_info('API -> set_gripper_enable(enable={}) -> code={}'.format(enable, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def _set_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_set_mode(mode)
        self.log_api_info('API -> set_gripper_mode(mode={}) -> code={}'.format(mode, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def _set_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_set_posspd(speed)
        self.log_api_info('API -> set_gripper_speed(speed={}) -> code={}'.format(speed, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def _get_gripper_position(self):
        ret = self.arm_cmd.gripper_get_pos()
        if self._check_code(ret[0]) != 0 or len(ret) <= 1:
            return ret[0], None
        elif ret[0] in [XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self.get_err_warn_code()
            if self.error_code == 19 or self.error_code == 28:
                return ret[0], None
            ret[0] = 0
        return ret[0], int(ret[1])

    @xarm_is_connected(_type='set')
    def _set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, **kwargs):
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move(is_stop=is_stop)
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if auto_enable:
            self.arm_cmd.gripper_set_en(True)
        if speed is not None:
            self.arm_cmd.gripper_set_posspd(speed)
        ret = self.arm_cmd.gripper_set_pos(pos)
        self.log_api_info('API -> set_gripper_position(pos={}) -> code={}'.format(pos, ret[0]), code=ret[0])
        if wait:
            is_add = True
            last_pos = 0
            _, p = self._get_gripper_position()
            if _ == 0 and p is not None:
                last_pos = int(p)
                if last_pos == pos:
                    return 0
                is_add = True if pos > last_pos else False
            count = 0
            count2 = 0
            if not timeout or not isinstance(timeout, (int, float)):
                timeout = 10
            expired = time.monotonic() + timeout
            while time.monotonic() < expired:
                _, p = self._get_gripper_position()
                if _ == 0 and p is not None:
                    cur_pos = int(p)
                    if abs(pos - cur_pos) <= 1:
                        last_pos = cur_pos
                        break
                    if is_add:
                        if cur_pos <= last_pos:
                            count += 1
                        elif cur_pos <= pos:
                            last_pos = cur_pos
                            count = 0
                            count2 = 0
                        else:
                            count2 += 1
                            if count2 >= 10:
                                break
                    else:
                        if cur_pos >= last_pos:
                            count += 1
                        elif cur_pos >= pos:
                            last_pos = cur_pos
                            count = 0
                            count2 = 0
                        else:
                            count2 += 1
                            if count2 >= 10:
                                break
                    if count >= 7:
                        # print('gripper target: {}, current: {}'.format(pos, cur_pos))
                        break
                    time.sleep(0.2)
                else:
                    return _
            # print('gripper, pos: {}, last: {}'.format(pos, last_pos))
            return ret[0]
        else:
            return ret[0]

    @xarm_is_connected(_type='get')
    def _get_gripper_err_code(self):
        ret = self.arm_cmd.gripper_get_errcode()
        if self._check_code(ret[0]) == 0:
            self._gripper_error_code = ret[1]
            if ret[0] == XCONF.UxbusState.ERR_CODE:
                self.get_err_warn_code()
                if self.error_code == 19 or self.error_code == 28:
                    print('gripper/tgpio error, code=C{}, G{}'.format(self.error_code, ret[1]))
                    return ret[0], ret[1]
            ret[0] = 0
            return ret[0], ret[1]
        return ret[0], 0

    @xarm_is_connected(_type='set')
    def _clean_gripper_error(self):
        ret = self.arm_cmd.gripper_clean_err()
        self.log_api_info('API -> clean_gripper_error -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def _set_gripper_zero(self):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        ret = self.arm_cmd.gripper_set_zero()
        self.log_api_info('API -> set_gripper_zero -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    ########################### Modbus Protocol #################################
    @xarm_is_connected(_type='set')
    def _set_modbus_gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_modbus_set_en(int(enable))
        _, err = self._get_modbus_gripper_err_code()
        self.log_api_info('API -> set_modbus_gripper_enable(enable={}) -> code={}, code2={}, err={}'.format(enable, ret[0], _, err), code=ret[0])
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if ret[0] == 0 and self.gripper_error_code == 0:
            self.gripper_is_enabled = True
        return ret[0] if self._gripper_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    def _set_modbus_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_modbus_set_mode(mode)
        _, err = self._get_modbus_gripper_err_code()
        self.log_api_info('API -> set_modbus_gripper_mode(mode={}) -> code={}, code2={}, err={}'.format(mode, ret[0], _, err), code=ret[0])
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        return ret[0] if self._gripper_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    def _set_modbus_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_modbus_set_posspd(speed)
        _, err = self._get_modbus_gripper_err_code()
        self.log_api_info('API -> set_modbus_gripper_speed(speed={}) -> code={}, code2={}, err={}'.format(speed, ret[0], _, err), code=ret[0])
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if ret[0] == 0 and self.gripper_error_code == 0:
            self.gripper_speed = speed
        return ret[0] if self._gripper_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='get')
    def _get_modbus_gripper_position(self):
        ret = self.arm_cmd.gripper_modbus_get_pos()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        _, err = self._get_modbus_gripper_err_code()
        if self._gripper_error_code != 0:
            return APIState.END_EFFECTOR_HAS_FAULT
        if ret[0] != 0 or len(ret) <= 1:
            return ret[0], None
        elif _ == 0 and err == 0:
            return ret[0], int(ret[1])
        else:
            return ret[0], None
            # return _ if err == 0 else XCONF.UxbusState.ERR_CODE, None

    def __check_gripper_position(self, target_pos, timeout=None):
        is_add = True
        last_pos = 0
        _, p = self._get_modbus_gripper_position()
        if _ == 0 and p is not None:
            last_pos = int(p)
            if last_pos == target_pos:
                return 0
            is_add = True if target_pos > last_pos else False
        count = 0
        count2 = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 10
        expired = time.monotonic() + timeout
        failed_cnt = 0
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.monotonic() < expired:
            _, p = self._get_modbus_gripper_position()
            if self._gripper_error_code != 0:
                print('xArm Gripper ErrorCode: {}'.format(self._gripper_error_code))
                return APIState.END_EFFECTOR_HAS_FAULT
            failed_cnt = 0 if _ == 0 and p is not None else failed_cnt + 1
            if _ == 0 and p is not None:
                cur_pos = int(p)
                if abs(target_pos - cur_pos) <= 1:
                    return 0
                if is_add:
                    if cur_pos <= last_pos:
                        count += 1
                    elif cur_pos <= target_pos:
                        last_pos = cur_pos
                        count = 0
                        count2 = 0
                    else:
                        count2 += 1
                        if count2 >= 10:
                            return 0
                else:
                    if cur_pos >= last_pos:
                        count += 1
                    elif cur_pos >= target_pos:
                        last_pos = cur_pos
                        count = 0
                        count2 = 0
                    else:
                        count2 += 1
                        if count2 >= 10:
                            return 0
                if count >= 8:
                    return 0
            else:
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.2)
        return code

    def __check_gripper_status(self, timeout=None):
        start_move = False
        not_start_move_cnt = 0
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 10
        expired = time.monotonic() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.monotonic() < expired:
            _, status = self.get_gripper_status()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                if status & 0x03 == 0 or status & 0x03 == 2:
                    if start_move:
                        return 0
                    else:
                        not_start_move_cnt += 1
                        if not_start_move_cnt > 20:
                            return 0
                elif not start_move:
                    not_start_move_cnt = 0
                    start_move = True
            else:
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.1)
        return code

    def check_catch_gripper_status(self, timeout=None):
        start_move = False
        not_start_move_cnt = 0
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 10
        expired = time.monotonic() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.monotonic() < expired:
            _, status = self.get_gripper_status()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                if status & 0x03 == 2:
                    if start_move:
                        return 0
                    else:
                        not_start_move_cnt += 1
                        if not_start_move_cnt > 2:
                            return 0
                elif not start_move:
                    not_start_move_cnt = 0
                    start_move = True
            else:
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.1)
        return code

    @xarm_is_connected(_type='set')
    def _set_modbus_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, **kwargs):
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move(is_stop=is_stop)
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if auto_enable and not self.gripper_is_enabled:
            ret = self.arm_cmd.gripper_modbus_set_en(True)
            ret[0] = self._check_modbus_code(ret, only_check_code=True)
            if ret[0] == 0:
                self.gripper_is_enabled = True
        if speed is not None and self.gripper_speed != speed:
            ret = self.arm_cmd.gripper_modbus_set_posspd(speed)
            ret[0] = self._check_modbus_code(ret, only_check_code=True)
            if ret[0] == 0:
                self.gripper_speed = speed
        ret = self.arm_cmd.gripper_modbus_set_pos(pos)
        self.log_api_info('API -> set_modbus_gripper_position(pos={}) -> code={}'.format(pos, ret[0]), code=ret[0])
        _, err = self._get_modbus_gripper_err_code()
        if self._gripper_error_code != 0:
            print('xArm Gripper ErrorCode: {}'.format(self._gripper_error_code))
            return APIState.END_EFFECTOR_HAS_FAULT
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if wait and ret[0] == 0:
            if self.gripper_is_support_status:
                return self.__check_gripper_status(timeout=timeout)
            else:
                return self.__check_gripper_position(pos, timeout=timeout)
        return ret[0]

    @xarm_is_connected(_type='get')
    def _get_modbus_gripper_err_code(self):
        ret = self.arm_cmd.gripper_modbus_get_errcode()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if ret[0] == 0:
            if ret[1] < 128:
                self._gripper_error_code = ret[1]
                if self._gripper_error_code != 0:
                    self.gripper_is_enabled = False
                    self.gripper_speed = 0
            return ret[0], ret[1]
        return ret[0], 0

    @xarm_is_connected(_type='set')
    def _clean_modbus_gripper_error(self):
        ret = self.arm_cmd.gripper_modbus_clean_err()
        self._gripper_error_code = 0
        _, err = self._get_modbus_gripper_err_code()
        self.log_api_info('API -> clean_modbus_gripper_error -> code={}, code2={}, err={}'.format(ret[0], _, err), code=ret[0])
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        return ret[0] if self._gripper_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    def _set_modbus_gripper_zero(self):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        ret = self.arm_cmd.gripper_modbus_set_zero()
        _, err = self._get_modbus_gripper_err_code()
        self.log_api_info('API -> set_modbus_gripper_zero -> code={}, code2={}, err={}'.format(ret[0], _, err), code=ret[0])
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        return ret[0] if self._gripper_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    def __bio_gripper_send_modbus(self, data_frame, min_res_len=0):
        code = self.checkset_modbus_baud(self._default_bio_baud)
        if code != 0:
            return code, []
        return self.set_rs485_data(data_frame, min_res_len=min_res_len, ignore_log=True)

    def __bio_gripper_wait_motion_completed(self, timeout=5, **kwargs):
        failed_cnt = 0
        expired = time.monotonic() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        check_detected = kwargs.get('check_detected', False)
        while time.monotonic() < expired:
            _, status = self.get_bio_gripper_status()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                code = code if (status & 0x03) == XCONF.BioGripperState.IS_MOTION \
                    else APIState.END_EFFECTOR_HAS_FAULT if (status & 0x03) == XCONF.BioGripperState.IS_FAULT \
                    else 0 if not check_detected or (status & 0x03) == XCONF.BioGripperState.IS_DETECTED else code
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.1)
        if self.bio_gripper_error_code != 0:
            print('BIO Gripper ErrorCode: {}'.format(self.bio_gripper_error_code))
        if code == 0 and not self.bio_gripper_is_enabled:
            code = APIState.END_EFFECTOR_NOT_ENABLED
        return code

    def __bio_gripper_wait_enable_completed(self, timeout=3):
        failed_cnt = 0
        expired = time.monotonic() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while time.monotonic() < expired:
            _, status = self.get_bio_gripper_status()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                code = 0 if self.bio_gripper_is_enabled else code
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.1)
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=False)
    def check_bio_gripper_is_catch(self, timeout=3):
        return self.__bio_gripper_wait_motion_completed(timeout=timeout, check_detected=True) == 0

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_bio_gripper_enable(self, enable, wait=True, timeout=3):
        data_frame = [0x08, 0x06, 0x01, 0x00, 0x00, int(enable)]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        if code == 0 and enable and wait:
            code = self.__bio_gripper_wait_enable_completed(timeout=timeout)
        self.log_api_info('API -> set_bio_gripper_enable(enable={}, wait={}, timeout={}) ->code={}'.format(enable, wait, timeout, code), code=code)
        # self.bio_gripper_is_enabled = True if code == 0 else self.bio_gripper_is_enabled
        self.get_bio_gripper_sn()
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=0)
    def get_bio_gripper_sn(self):
        code, ret = self.get_bio_gripper_register(address=0x0B10, number_of_registers=16)
        if code == APIState.MODBUS_ERR_LENG:
            self._bio_gripper_version = 1
            return code, ret
        elif code != 0:
            self._bio_gripper_version = 0
            return code, ''
        else:
            self._bio_gripper_version = 2
            # return code, ''.join(list(map(chr, ret[5:])))
            ret_list = [int(str(ret[i]) + str(ret[i+1])) for i in range(3, 31, 2)]
            return code, ''.join(list(map(chr, ret_list)))

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_bio_gripper_control_mode(self, mode):
        data_frame = [0x08, 0x06, 0x11, 0x0A, 0x00, mode]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        time.sleep(0.2)
        # reset mcu
        data_frame = [0x08, 0x06, 0x06, 0x07, 0x00, 0x01]
        self.__bio_gripper_send_modbus(data_frame, 6)
        # wait mcu reboot
        time.sleep(0.8)
        self.bio_gripper_speed = 0
        self.bio_gripper_force = 0
        self._bio_gripper_version = 0
        self._bio_gripper_mode = -1
        return code

    def __check_bio_gripper_version(self):
        if self._bio_gripper_version == 0:
            self.get_bio_gripper_sn()
        if self._bio_gripper_version == 2 and self._bio_gripper_mode == -1:
            _, self._bio_gripper_mode = self.get_bio_gripper_control_mode()

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=0)
    def get_bio_gripper_control_mode(self):
        code, ret = self.get_bio_gripper_register(address=0x010A, number_of_registers=1)
        mode = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        return code, mode

    # @xarm_is_connected(_type='set')
    # @xarm_is_not_simulation_mode(ret=0)
    # def set_bio_gripper_zero(self):
    #     data_frame = [0x08, 0x06, 0x08, 0x0C, 0x00, 0x01]
    #     code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
    #     return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_bio_gripper_force(self, force):
        force = 1 if force < 1 else 100 if force > 100 else force
        data_frame = [0x08, 0x06, 0x05, 0x06, 0x00, force]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        self.log_api_info('API -> set_bio_gripper_force(force={}) ->code={}'.format(force, code), code=code)
        self.bio_gripper_force = force if code == 0 else self.bio_gripper_force
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_bio_gripper_speed(self, speed):
        data_frame = [0x08, 0x06, 0x03, 0x03, speed // 256 % 256, speed % 256]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        self.log_api_info('API -> set_bio_gripper_speed(speed={}) ->code={}'.format(speed, code), code=code)
        self.bio_gripper_speed = speed if code == 0 else self.bio_gripper_speed
        return code

    @xarm_is_connected(_type='set')
    def set_bio_gripper_position(self, pos, speed=0, force=50, wait=True, timeout=5, **kwargs):
        if kwargs.get('is_g2', True):
            return self.set_bio_gripper_g2_position(pos, speed=speed, force=force, wait=wait, timeout=timeout, **kwargs)
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move(is_stop=is_stop)
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0
        self.get_bio_gripper_status()
        if not self.bio_gripper_is_enabled:
            self.set_bio_gripper_enable(True)
        speed = min(speed, 4000)
        speed = speed if speed > 0 else 2000
        if speed > 0 and speed != self.bio_gripper_speed:
            self.set_bio_gripper_speed(speed)

        self.__check_bio_gripper_version()
        if self._bio_gripper_mode == 1:
            pos = int(pos * 3.7342 - 265.13)
        if self._bio_gripper_version == 2:
            force = min(force, 100)
            if force > 0 and force != self.bio_gripper_force:
                self.set_bio_gripper_force(force)

        data_frame = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04]
        data_frame.extend(list(struct.pack('>i', pos)))
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        if code == 0 and wait:
            code = self.__bio_gripper_wait_motion_completed(timeout=timeout)
        self.log_api_info(
            'API -> set_bio_gripper_position(pos={}, wait={}, timeout={}) ->code={}'.format(pos, wait, timeout, code),
            code=code)
        return code

    @xarm_is_connected(_type='set')
    def open_bio_gripper(self, speed=0, wait=True, timeout=5, **kwargs):
        self.__check_bio_gripper_version()
        pos = 150 if self._bio_gripper_version == 2 else 130
        return self.set_bio_gripper_position(pos, speed=speed, wait=wait, timeout=timeout, is_g2=False, **kwargs)

    @xarm_is_connected(_type='set')
    def close_bio_gripper(self, speed=0, wait=True, timeout=5, **kwargs):
        self.__check_bio_gripper_version()
        pos = 71 if self._bio_gripper_version == 2 else 50
        return self.set_bio_gripper_position(pos, speed=speed, wait=wait, timeout=timeout, is_g2=False, **kwargs)

    @xarm_is_connected(_type='get')
    def get_bio_gripper_register(self, address=0x00, number_of_registers=1):
        data_frame = [0x08, 0x03]
        data_frame.extend(list(struct.pack('>h', address)))
        data_frame.extend(list(struct.pack('>h', number_of_registers)))
        return self.__bio_gripper_send_modbus(data_frame, 3 + 2 * number_of_registers)

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_bio_gripper_status(self):
        code, ret = self.get_bio_gripper_register(address=0x00)
        status = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        if code == 0:
            if (status & 0x03) == XCONF.BioGripperState.IS_FAULT:
                self.get_bio_gripper_error()
            else:
                self.bio_gripper_error_code = 0
            if ((status >> 2) & 0x03) == XCONF.BioGripperState.IS_ENABLED:
                self.bio_gripper_is_enabled = True
            else:
                self.bio_gripper_is_enabled = False
        return code, status

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_bio_gripper_error(self):
        code, ret = self.get_bio_gripper_register(address=0x0F)
        error_code = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        if code == 0:
            self.bio_gripper_error_code = error_code
        return code, error_code

    @xarm_is_not_simulation_mode(ret=(0, '*.*.*'))
    def get_bio_gripper_version(self):
        code, ret = self.get_bio_gripper_register(address=0x0801, number_of_registers=0x03)
        versions = ['*', '*', '*']
        if code == 0:
            versions[0] = convert.bytes_to_u16(ret[3:5])
            versions[1] = convert.bytes_to_u16(ret[5:7])
            versions[2] = convert.bytes_to_u16(ret[7:9])
        return code, '.'.join(map(str, versions))

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_bio_gripper_g2_position(self, **kwargs):
        self.__check_bio_gripper_version()
        code, ret = self.get_bio_gripper_register(address=(0x0702 if self._bio_gripper_version == 2 else 0x0700), number_of_registers=2)
        bio_position = convert.bytes_to_int32(ret[3:])
        # # bio_position = (ret[-4] * 256**3 + ret[-3] * 256**2 + ret[-2] * 256 + ret[-1]) if code == 0 else -1
        if self._bio_gripper_version == 2:
            bio_position = int(round((bio_position + 265.13) / 3.7342, 0))
            bio_position = min(max(bio_position, 71), 150)
        else:
            bio_position = 150 if bio_position > 100 else 71
        return code, bio_position

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_bio_gripper_speed(self):
        code, ret = self.get_bio_gripper_register(address=0x0303)
        bio_speed = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        return code, bio_speed

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_bio_gripper_force(self):
        code, ret = self.get_bio_gripper_register(address=0x0506)
        bio_force = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        return code, bio_force

    @xarm_is_connected(_type='set')
    def clean_bio_gripper_error(self):
        data_frame = [0x08, 0x06, 0x00, 0x0F, 0x00, 0x00]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        self.log_api_info('API -> clean_bio_gripper_error -> code={}'.format(code), code=code)
        self.get_bio_gripper_status()
        return code

    @xarm_is_connected(_type='set')
    def open_lite6_gripper(self, sync=True):
        code1 = self.set_tgpio_digital(0, 1, sync=sync)
        code2 = self.set_tgpio_digital(1, 0, sync=sync)
        return code1 if code2 == 0 else code2

    @xarm_is_connected(_type='set')
    def close_lite6_gripper(self, sync=True):
        code1 = self.set_tgpio_digital(0, 0, sync=sync)
        code2 = self.set_tgpio_digital(1, 1, sync=sync)
        return code1 if code2 == 0 else code2

    @xarm_is_connected(_type='set')
    def stop_lite6_gripper(self, sync=True):
        code1 = self.set_tgpio_digital(0, 0, sync=sync)
        code2 = self.set_tgpio_digital(1, 0, sync=sync)
        return code1 if code2 == 0 else code2
    
    def __gripper_g2_send_modbus(self, data_frame, min_res_len=0):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code, []
        return self.set_rs485_data(data_frame, min_res_len=min_res_len, ignore_log=True)
    
    @xarm_is_connected(_type='get')
    def get_gripper_g2_register(self, address, number_of_registers=1):
        data_frame = [0x08, 0x03]
        data_frame.extend(list(struct.pack('>h', address)))
        data_frame.extend(list(struct.pack('>h', number_of_registers)))
        return self.__gripper_g2_send_modbus(data_frame, 3 + 2 * number_of_registers)
    
    def get_gripper_g2_position(self):
        code, pulse = self.get_gripper_position()
        if code != 0:
            return code, pulse
        pos = math.sin(math.radians(pulse / 18.28 - 8.33)) * 110 + 16
        return code, int(pos)
    
    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_gripper_g2_speed(self):
        code, ret = self.get_gripper_g2_register(address=0x0303, number_of_registers=1)
        if code != 0:
            return code, ret
        pulse_speed = ret[-2] * 256 + ret[-1]
        speed = ((pulse_speed * 0.4 - 140) * 9.88235) / 60
        speed = int(Decimal(str(speed)).quantize(Decimal('0'), rounding=ROUND_HALF_UP))
        return code, speed

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_gripper_g2_force(self):
        code, ret = self.get_gripper_g2_register(address=0x0500, number_of_registers=1)
        if code != 0:
            return code, ret
        force = ret[-2] * 256 + ret[-1]
        return code, force
    
    @xarm_is_connected(_type='set')
    def set_gripper_g2_position(self, pos, speed=100, force=50, wait=False, timeout=5, **kwargs):
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move(is_stop=is_stop)
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        pos = min(max(0, pos), 84)
        speed = min(max(15, speed), 225)
        force = min(max(1, force), 100)

        pos = int((math.degrees(math.asin((pos - 16) / 110)) + 8.33) * 18.28)
        speed = int(((speed * 60) / 9.88235 + 140) / 0.4)

        data_frame = [0x08, 0x10, 0x0C, 0x00, 0x00, 0x05, 0x0A, 0x00, 0x01]
        data_frame.extend(list(struct.pack('>h', speed))) # speed // 256 % 256, speed % 256
        data_frame.extend(list(struct.pack('>h', force))) # force // 256 % 256, force % 256
        data_frame.extend(list(struct.pack('>i', pos)))
        ret = self.set_rs485_data(data_frame, min_res_len=6, ignore_log=True)
        self.log_api_info('API -> set_g2_gripper_position(pos={}, speed={}, force={}) -> code={}'.format(pos, speed, force, ret[0]), code=ret[0])
        _, err = self._get_modbus_gripper_err_code()
        if self._gripper_error_code != 0:
            print('xArm Gripper G2 ErrorCode: {}'.format(self._gripper_error_code))
            return APIState.END_EFFECTOR_HAS_FAULT
        if wait and ret[0] == 0:
            if self.gripper_is_support_status:
                return self.__check_gripper_status(timeout=timeout)
            else:
                return self.__check_gripper_position(pos, timeout=timeout)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_bio_gripper_g2_position(self, pos, speed=2000, force=100, wait=True, timeout=5, **kwargs):
        if kwargs.pop('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move(is_stop=is_stop)
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0

        pos = min(max(71, pos), 150)
        speed = min(max(500, speed), 4000)
        force = min(max(1, force), 100)

        self.__check_bio_gripper_version()
        pos_pluse = pos
        if self._bio_gripper_mode == 1:
            pos_pluse = int(pos * 3.7342 - 265.13)

        data_frame = [0x08, 0x10, 0x0C, 0x00, 0x00, 0x05, 0x0A, 0x00, 0x01]
        data_frame.extend(list(struct.pack('>h', speed))) # speed // 256 % 256, speed % 256
        data_frame.extend(list(struct.pack('>h', force))) # force // 256 % 256, force % 256
        data_frame.extend(list(struct.pack('>i', pos_pluse)))
        code, res = self.__bio_gripper_send_modbus(data_frame, 6)
        if code == 0 and wait:
            code = self.__bio_gripper_wait_motion_completed(timeout=timeout)
        elif code == APIState.MODBUS_ERR_LENG and len(res) >= 3 and res[2] == 2:
            # res[2] == 2: Illegal data address, BIO version does not support
            return self.set_bio_gripper_position(pos, speed, force, wait, timeout, wait_motion=False, is_g2=False, **kwargs)
        self.log_api_info(
            'API -> set_bio_gripper_g2_position(pos={}, speed={}, force={}, wait={}, timeout={}) -> code={}'.format(pos, speed, force, wait, timeout, code),
            code=code)
        return code

    def __dhpgc_gripper_send_modbus(self, data_frame, min_res_len=0):
        code = self.checkset_modbus_baud(self._default_dhpgc_gripper_baud)
        if code != 0:
            return code, []
        return self.set_rs485_data(data_frame, min_res_len=min_res_len, ignore_log=True)

    @xarm_is_connected(_type='get')
    def get_dhpgc_gripper_register(self, address=0x00, number_of_registers=1):
        data_frame = [0x01, 0x03]
        data_frame.extend(list(struct.pack('>h', address)))
        data_frame.extend(list(struct.pack('>h', number_of_registers)))
        return self.__dhpgc_gripper_send_modbus(data_frame, 3 + 2 * number_of_registers)
    
    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, [0]))
    def get_dhpgc_gripper_status(self, address, get_status_type):
        code, ret = self.get_dhpgc_gripper_register(address=address)
        if code == 0 and len(ret) >= 5:
            gripper_status_reg = ret[4]
            if get_status_type == 'get_activation':
                if gripper_status_reg == 1:
                    self.dhpgc_is_activated = True
                else:
                    self.dhpgc_is_activated = False
            else:
                self.dhpgc_picked_status = gripper_status_reg
        return code, ret

    def __dhpgc_wait_activation_completed(self, timeout=3):
        failed_cnt = 0
        expired = time.monotonic() + timeout if timeout is not None and timeout > 0 else 0
        code = APIState.WAIT_FINISH_TIMEOUT
        while expired == 0 or time.monotonic() < expired:
            _, ret = self.get_dhpgc_gripper_status(0x0200, 'get_activation')
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                code = 0 if self.dhpgc_is_activated else code
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.05)
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_dhpgc_gripper_activate(self, wait=True, timeout=3):
        data_frame = [0x01, 0x06, 0x01, 0x00, 0x00, 0x01]
        code, _ = self.__dhpgc_gripper_send_modbus(data_frame, 6)
        if wait and code == 0:
            code = self.__dhpgc_wait_activation_completed(timeout)
        self.log_api_info(
            'API -> set_dhpgc_gripper_activate(wait={}, timeout={}) ->code={}'.format(wait, timeout, code), code=code)
        if code == 0:
            self.dhpgc_is_activated = True
        return code

    def __dhpgc_wait_motion_completed(self, timeout=5, **kwargs):
        failed_cnt = 0
        expired = time.monotonic() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        check_detected = kwargs.get('check_detected', False)
        while expired == 0 or time.monotonic() < expired:
            _, ret = self.get_dhpgc_gripper_status(0x0201, 'get_picked')
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                code = code if self.dhpgc_picked_status == 0 else 0 if \
                    (check_detected and self.dhpgc_picked_status == 2) or not check_detected else code
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if \
                    failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.05)
        if code == 0 and not self.dhpgc_is_activated:
            code = APIState.END_EFFECTOR_NOT_ENABLED
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_dhpgc_gripper_speed(self, speed):
        force = 1 if speed < 1 else 100 if speed > 100 else speed
        data_frame = [0x01, 0x06, 0x01, 0x04, 0x00, speed]
        code, _ = self.__dhpgc_gripper_send_modbus(data_frame, 6)
        self.log_api_info('API -> set_dhpgc_gripper_speed(speed={}) ->code={}'.format(speed, code), code=code)
        self.dhpgc_gripper_speed = speed if code == 0 else self.dhpgc_gripper_speed
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_dhpgc_gripper_force(self, force):
        force = 20 if force < 20 else 100 if force > 100 else force
        data_frame = [0x01, 0x06, 0x01, 0x01, 0x00, force]
        code, _ = self.__dhpgc_gripper_send_modbus(data_frame, 6)
        self.log_api_info('API -> set_dhpgc_gripper_force(force={}) ->code={}'.format(force, code), code=code)
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_dhpgc_gripper_position(self):
        code, ret = self.get_dhpgc_gripper_register(address=0x0202)
        dhpgc_gripper_position = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        return code, dhpgc_gripper_position

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_dhpgc_gripper_speed(self):
        code, ret = self.get_dhpgc_gripper_register(address=0x0104)
        dhpgc_gripper_speed = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        return code, dhpgc_gripper_speed

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_dhpgc_gripper_force(self):
        code, ret = self.get_dhpgc_gripper_register(address=0x0101)
        dhpgc_gripper_force = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        return code, dhpgc_gripper_force

    @xarm_is_connected(_type='set')
    def set_dhpgc_gripper_position(self, pos, speed=50, force=50, wait=True, timeout=5, **kwargs):
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move(is_stop=is_stop)
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0

        code, _ = self.get_dhpgc_gripper_status(0x0200, 'get_activation')
        if code == 0 and not self.dhpgc_is_activated:
            self.set_dhpgc_gripper_activate(wait=True)
        elif code != 0:
            return code

        speed = min(max(1, speed), 100)
        force = min(max(20, force), 100)
        if speed != self.dhpgc_gripper_speed:
            self.set_dhpgc_gripper_speed(speed)
        self.set_dhpgc_gripper_force(force)

        pos = min(max(0, pos), 1000)
        data_frame = [0x01, 0x06, 0x01, 0x03]
        data_frame.extend(list(struct.pack('>h', pos)))
        code, _ = self.__dhpgc_gripper_send_modbus(data_frame, 6)
        if wait and code == 0:
            code = self.__dhpgc_wait_motion_completed(timeout, **kwargs)
        self.log_api_info('API -> set_dhpgc_gripper_position(pos={}, wait={}, timeout={}) ->code={}'.format(
            pos, wait, timeout, code), code=code)
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=False)
    def check_dhpgc_gripper_is_catch(self, timeout=3):
        return self.__dhpgc_wait_motion_completed(timeout=timeout, check_detected=True) == 0

    def __rh56_finger_send_modbus(self, data_frame, min_res_len=0):
        code = self.checkset_modbus_baud(self._default_rh56_finger_baud)
        if code != 0:
            return code, []
        return self.set_rs485_data(data_frame, min_res_len=min_res_len, ignore_log=False)

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, [0]))
    def get_rh56_finger_status(self, finger_id, data_frame, get_status_type):
        code, ret = self.__rh56_finger_send_modbus(data_frame, 5)
        if code == 0 and len(ret) >= 5:
            if get_status_type == 'get_finger_status':
               self.rh56_finger_status[finger_id-1] = (ret[3] << 8) + ret[4]
            elif get_status_type == 'get_finger_pos':
                self.rh56_finger_pos[finger_id-1]  = (ret[3] << 8) + ret[4]
            elif get_status_type == 'get_finger_speed':
                self.rh56_finger_speed[finger_id-1]  = (ret[3] << 8) + ret[4]
            elif get_status_type == 'get_finger_force':
                self.rh56_finger_force[finger_id-1]  = (ret[3] << 8) + ret[4]
        return code, ret

    def __rh56_finger_wait_motion_completed(self, finger_id, data_frame, timeout=5, **kwargs):
        failed_cnt = 0
        expired = time.monotonic() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        check_detected = kwargs.get('check_detected', False)
        while expired == 0 or time.monotonic() < expired:
            _, ret = self.get_rh56_finger_status(finger_id, data_frame, 'get_finger_status')
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                code = code if (check_detected and self.rh56_finger_status[finger_id-1] in [0, 1]) else 0 if \
                    (check_detected and self.rh56_finger_status[finger_id-1]  == 2) or not check_detected else APIState.END_EFFECTOR_HAS_FAULT
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.05)
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_rh56_finger_speed(self, finger_id, speed):
        speed = min(max(round(speed), 0), 1000)
        addr = SPEED_ADDR[finger_id]
        data_frame = [HAND_ID, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (speed >> 8) & 0xFF, speed & 0xFF]
        code, _ = self.__rh56_finger_send_modbus(data_frame, 6)
        self.log_api_info('API -> set_rh56_finger_speed(finger_id={}, speed={}) ->code={}'.format(finger_id, speed,
                                                                                                  code), code=code)
        self.rh56_finger_speed[finger_id-1] = speed if code == 0 else self.rh56_finger_speed[finger_id-1]
        return code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def set_rh56_finger_force(self, finger_id, force):
        force = min(max(round(force), 0), 1000)
        addr = FORCE_ADDR[finger_id]
        data_frame = [HAND_ID, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (force >> 8) & 0xFF, force & 0xFF]
        code, _ = self.__rh56_finger_send_modbus(data_frame, 6)
        self.log_api_info('API -> set_rh56_finger_force(finger_id={}, force={}) ->code={}'.format(finger_id, force,
                                                                                                  code), code=code)
        return code

    @xarm_is_connected(_type='set')
    def set_rh56_finger_position(self, finger_id, pos, speed=500, force=500, wait=True, timeout=5, **kwargs):
        # if kwargs.get('wait_motion', True):
        #     has_error = self.error_code != 0
        #     is_stop = self.is_stop
        #     code = self.wait_move()
        #     if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
        #             or (has_error and code == APIState.HAS_ERROR)):
        #         return code
        if self.check_is_simulation_robot():
            return 0
        if speed != self.rh56_finger_speed[finger_id-1]:
            code = self.set_rh56_finger_speed(finger_id, speed)
            if code != 0 :
                return code

        code = self.set_rh56_finger_force(finger_id, force)
        if code != 0:
            return code

        pos = min(max(round(pos), 0), 1000) if pos > -1 else 0xFFFF
        addr = FINGER_ADDR[finger_id]
        data_frame = [HAND_ID, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF]
        code, _ = self.__rh56_finger_send_modbus(data_frame, 6)
        if wait and code == 0:
            status_addr = FINGER_STATUS_ADDR[finger_id]
            data_frame = [HAND_ID, 0x03, (status_addr >> 8) & 0xFF, status_addr & 0xFF, 0x00, 0x01]
            self.__rh56_finger_wait_motion_completed(finger_id, data_frame, timeout=timeout, check_detected=True)
        self.log_api_info('API -> set_rh56_finger_position(finger_id={}, pos={}, wait={}, timeout={}) ->code={}'.format(
            finger_id, pos, wait, timeout, code), code=code)
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_rh56_finger_position(self, finger_id):
        pos_addr = GET_POS_ADDR[finger_id]
        data_frame = [HAND_ID, 0x03, (pos_addr >> 8) & 0xFF, pos_addr & 0xFF, 0x00, 0x01]
        code, ret = self.get_rh56_finger_status(finger_id, data_frame, 'get_finger_pos')
        if code != 0:
            return code, 0
        elif len(ret) < 5:
            return APIState.UxbusState.INVALID, 0
        return code, self.rh56_finger_pos[finger_id-1]

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_rh56_finger_speed(self, finger_id):
        speed_addr = SPEED_ADDR[finger_id]
        data_frame = [HAND_ID, 0x03, (speed_addr >> 8) & 0xFF, speed_addr & 0xFF, 0x00, 0x01]
        code, ret = self.get_rh56_finger_status(finger_id, data_frame, 'get_finger_speed')
        if code != 0:
            return code, 0
        elif len(ret) < 5:
            return APIState.UxbusState.INVALID, 0
        return code, self.rh56_finger_speed[finger_id-1]

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, [0]))
    def get_rh56_finger_force(self, finger_id):
        force_addr = FORCE_ADDR[finger_id]
        data_frame = [HAND_ID, 0x03, (force_addr >> 8) & 0xFF, force_addr & 0xFF, 0x00, 0x01]
        code, ret = self.get_rh56_finger_status(finger_id, data_frame, 'get_finger_force')
        if code != 0:
            return code, 0
        elif len(ret) < 5:
            return APIState.UxbusState.INVALID, 0
        return code, self.rh56_finger_force[finger_id-1]

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, [0]))
    def get_rh56_hand_position(self):
        finger_pos_list = []
        for finger_id in range(1, 7):
            code, finger_pos = self.get_rh56_finger_position(finger_id)
            if code != 0:
                return code, []
            finger_pos_list.append(finger_pos)
        return code, finger_pos_list

    @xarm_is_connected(_type='set')
    def set_rh56_hand_position(self, pos, speed=1000, force=500, wait=True, timeout=5):
        _, finger_pos_list = self.get_rh56_hand_position()
        average_finger_pos = sum(finger_pos_list) / 5
        if pos <= average_finger_pos:
            finger_order = [6, 1, 2, 3, 4, 5]
        else:
            finger_order = [6, 5, 4, 3, 2, 1]

        pos = min(max(0, pos), 1000)
        speed = min(max(0, speed), 1000)
        force = min(max(0, force), 1000)

        for finger_id in finger_order:
            code = self.set_rh56_finger_position(finger_id, pos if finger_id != 6 else 1000, speed, force, wait=False)
            if code != 0:
                return code
            time.sleep(0.01)
        if wait:
            for finger_id in finger_order:
                if self.error_code != 0:
                    return self.error_code
                status_addr = FINGER_STATUS_ADDR[finger_id]
                data_frame = [HAND_ID, 0x03, (status_addr >> 8) & 0xFF, status_addr & 0xFF, 0x00, 0x01]
                self.__rh56_finger_wait_motion_completed(finger_id, data_frame, timeout=timeout, check_detected=True)
        return code