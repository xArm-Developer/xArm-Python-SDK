#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
import struct
from ..core.config.x_config import XCONF
from ..core.utils.log import logger
from ..core.utils import convert
from .code import APIState
from .gpio import GPIO
from .decorator import xarm_is_connected, xarm_wait_until_not_pause, xarm_is_not_simulation_mode


class Gripper(GPIO):
    def __init__(self):
        super(Gripper, self).__init__()
        self._gripper_error_code = 0

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
            return self._get_gripper_position()

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
    def set_gripper_status(self, status, delay_sec=0):
        code = self.checkset_modbus_baud(self._default_gripper_baud)
        if code != 0:
            return code
        if status:
            code1 = self.set_tgpio_digital(ionum=0, value=1, delay_sec=delay_sec)
            code2 = self.set_tgpio_digital(ionum=1, value=0, delay_sec=delay_sec)
        else:
            code1 = self.set_tgpio_digital(ionum=0, value=0, delay_sec=delay_sec)
            code2 = self.set_tgpio_digital(ionum=1, value=1, delay_sec=delay_sec)
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
            code = self.wait_move()
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

    @xarm_is_connected(_type='set')
    def _set_modbus_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, **kwargs):
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move()
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
        return self.getset_tgpio_modbus_data(data_frame, min_res_len=min_res_len, ignore_log=True)

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
    def set_bio_gripper_position(self, pos, speed=0, wait=True, timeout=5, **kwargs):
        if kwargs.get('wait_motion', True):
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move()
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0
        if kwargs.get('auto_enable', False) and not self.bio_gripper_is_enabled:
            self.set_bio_gripper_enable(True)
        if speed > 0 and speed != self.bio_gripper_speed:
            self.set_bio_gripper_speed(speed)
        data_frame = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04]
        data_frame.extend(list(struct.pack('>i', pos)))
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        if code == 0 and wait:
            code = self.__bio_gripper_wait_motion_completed(timeout=timeout)
        self.log_api_info('API -> set_bio_gripper_position(pos={}, wait={}, timeout={}) ->code={}'.format(pos, wait, timeout, code), code=code)
        return code

    @xarm_is_connected(_type='set')
    def open_bio_gripper(self, speed=0, wait=True, timeout=5, **kwargs):
        return self.set_bio_gripper_position(130, speed=speed, wait=wait, timeout=timeout, **kwargs)
        # if kwargs.get('auto_enable', False) and not self.bio_gripper_is_enabled:
        #     self.set_bio_gripper_enable(True)
        # if speed > 0 and speed != self.bio_gripper_speed:
        #     self.set_bio_gripper_speed(speed)
        # data_frame = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x80, 0x00, 0x00, 0x64]
        # code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        # if code == 0 and wait:
        #     code = self.__bio_gripper_wait_motion_completed(timeout=timeout, **kwargs)
        # self.log_api_info('API -> open_bio_gripper(wait={}, timeout={}) ->code={}'.format(wait, timeout, code), code=code)
        # return code

    @xarm_is_connected(_type='set')
    def close_bio_gripper(self, speed=0, wait=True, timeout=5, **kwargs):
        return self.set_bio_gripper_position(50, speed=speed, wait=wait, timeout=timeout, **kwargs)
        # if kwargs.get('auto_enable', False):
        #     self.set_bio_gripper_enable(True)
        # if speed > 0 and speed != self.bio_gripper_speed:
        #     self.set_bio_gripper_speed(speed)
        # data_frame = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x64]
        # code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        # if code == 0 and wait:
        #     code = self.__bio_gripper_wait_motion_completed(timeout=timeout, **kwargs)
        # self.log_api_info('API -> close_bio_gripper(wait={}, timeout={}) ->code={}'.format(wait, timeout, code), code=code)
        # return code

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

    @xarm_is_connected(_type='set')
    def clean_bio_gripper_error(self):
        data_frame = [0x08, 0x06, 0x00, 0x0F, 0x00, 0x00]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        self.log_api_info('API -> clean_bio_gripper_error -> code={}'.format(code), code=code)
        self.get_bio_gripper_status()
        return code

    @xarm_is_connected(_type='set')
    def open_lite6_gripper(self):
        code1 = self.set_tgpio_digital(0, 1)
        code2 = self.set_tgpio_digital(1, 0)
        return code1 if code2 == 0 else code2

    @xarm_is_connected(_type='set')
    def close_lite6_gripper(self):
        code1 = self.set_tgpio_digital(0, 0)
        code2 = self.set_tgpio_digital(1, 1)
        return code1 if code2 == 0 else code2

    @xarm_is_connected(_type='set')
    def stop_lite6_gripper(self):
        code1 = self.set_tgpio_digital(0, 0)
        code2 = self.set_tgpio_digital(1, 0)
        return code1 if code2 == 0 else code2
