#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from .utils import xarm_is_connected, xarm_is_pause, check_modbus_baud
from ..core.config.x_config import XCONF
from ..core.utils.log import logger
from ..core.utils import convert
from .code import APIState
from .gpio import GPIO

GRIPPER_BAUD = 2000000


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

    @xarm_is_connected(_type='get')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='get', default='*.*.*')
    def get_gripper_version(self):
        versions = ['*', '*', '*']
        ret1 = self.arm_cmd.gripper_modbus_r16s(0x0801, 1)
        ret2 = self.arm_cmd.gripper_modbus_r16s(0x0802, 1)
        ret3 = self.arm_cmd.gripper_modbus_r16s(0x0803, 1)

        code = 0

        if ret1[0] == 0 and len(ret1) == 7:
            versions[0] = convert.bytes_to_u16(ret1[5:7])
        else:
            code = ret1[0]

        if ret2[0] == 0 and len(ret2) == 7:
            versions[1] = convert.bytes_to_u16(ret2[5:7])
        else:
            code = ret2[0]

        if ret3[0] == 0 and len(ret3) == 7:
            versions[2] = convert.bytes_to_u16(ret3[5:7])
        else:
            code = ret3[0]

        return code, '.'.join(map(str, versions))

    @xarm_is_connected(_type='set')
    def set_gripper_enable(self, enable, is_modbus=True):
        if is_modbus:
            return self._set_modbus_gripper_enable(enable)
        else:
            return self._set_gripper_enable(enable)

    @xarm_is_connected(_type='set')
    def set_gripper_mode(self, mode, is_modbus=True):
        if is_modbus:
            return self._set_modbus_gripper_mode(mode)
        else:
            return self._set_gripper_mode(mode)

    @xarm_is_connected(_type='set')
    def set_gripper_speed(self, speed, is_modbus=True):
        if is_modbus:
            return self._set_modbus_gripper_speed(speed)
        else:
            return self._set_gripper_speed(speed)

    @xarm_is_connected(_type='get')
    def get_gripper_position(self, is_modbus=True):
        if is_modbus:
            return self._get_modbus_gripper_position()
        else:
            return self._get_gripper_position()

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, is_modbus=True):
        if is_modbus:
            return self._set_modbus_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout)
        else:
            return self._set_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout)

    @xarm_is_connected(_type='get')
    def get_gripper_err_code(self, is_modbus=True):
        if is_modbus:
            return self._get_modbus_gripper_err_code()
        else:
            return self._get_gripper_position()

    @xarm_is_connected(_type='set')
    def clean_gripper_error(self, is_modbus=True):
        if is_modbus:
            return self._clean_modbus_gripper_error()
        else:
            return self._clean_gripper_error()

    @xarm_is_connected(_type='set')
    def set_gripper_zero(self, is_modbus=True):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        if is_modbus:
            return self._set_modbus_gripper_zero()
        else:
            return self._set_gripper_zero()

    @xarm_is_connected(_type='set')
    def set_gripper_status(self, status, delay_sec=0):
        if status:
            code1 = self.set_tgpio_digital(ionum=0, value=1, delay_sec=delay_sec)
            code2 = self.set_tgpio_digital(ionum=1, value=0, delay_sec=delay_sec)
        else:
            code1 = self.set_tgpio_digital(ionum=0, value=0, delay_sec=delay_sec)
            code2 = self.set_tgpio_digital(ionum=1, value=1, delay_sec=delay_sec)
        code = code1 if code2 == 0 else code2
        logger.info('API -> set_gripper_status -> ret={}, on={}, delay: {}'.format(code, status, delay_sec))
        return code

    ########################### Old Protocol #################################
    @xarm_is_connected(_type='set')
    def _set_gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_set_en(int(enable))
        logger.info(
            'API -> set_gripper_enable -> ret={}, enable={}'.format(ret[0], enable))
        return ret[0]

    @xarm_is_connected(_type='set')
    def _set_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_set_mode(mode)
        logger.info('API -> set_gripper_mode -> ret={}, mode={}'.format(ret[0], mode))
        return ret[0]

    @xarm_is_connected(_type='set')
    def _set_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_set_posspd(speed)
        logger.info('API -> set_gripper_speed -> ret={}, speed={}'.format(ret[0], speed))
        return ret[0]

    @xarm_is_connected(_type='get')
    def _get_gripper_position(self):
        ret = self.arm_cmd.gripper_get_pos()
        if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] or len(ret) <= 1:
            return ret[0], None
        elif ret[0] in [XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self.get_err_warn_code()
            if self.error_code == 19 or self.error_code == 28:
                return ret[0], None
            ret[0] = 0
        return ret[0], int(ret[1])

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def _set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):
        if auto_enable:
            self.arm_cmd.gripper_set_en(True)
        if speed is not None:
            self.arm_cmd.gripper_set_posspd(speed)
        ret = self.arm_cmd.gripper_set_pos(pos)
        logger.info('API -> set_gripper_position -> ret={}, pos={}'.format(ret[0], pos))
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
            start_time = time.time()
            if not timeout or not isinstance(timeout, (int, float)):
                timeout = 10
            while time.time() - start_time < timeout:
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
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
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
        logger.info('API -> clean_gripper_error -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def _set_gripper_zero(self):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        ret = self.arm_cmd.gripper_set_zero()
        logger.info('API -> set_gripper_zero -> ret={}'.format(ret[0]))
        return ret[0]

    ########################### Modbus Protocol #################################
    @xarm_is_connected(_type='set')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='set', default=None)
    def _set_modbus_gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_modbus_set_en(int(enable))
        _, err = self._get_modbus_gripper_err_code()
        logger.info('API -> set_modbus_gripper_enable -> ret={}, enable={}, ret2={}, err={}'.format(ret[0], enable, _, err))
        ret[0] = self._check_modbus_code(ret)
        if ret[0] == 0 and self.gripper_error_code == 0:
            self.gripper_is_enabled = True
        return ret[0]

    @xarm_is_connected(_type='set')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='set', default=None)
    def _set_modbus_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_modbus_set_mode(mode)
        _, err = self._get_modbus_gripper_err_code()
        logger.info('API -> set_modbus_gripper_mode -> ret={}, mode={}, ret2={}, err={}'.format(ret[0], mode, _, err))
        ret[0] = self._check_modbus_code(ret)
        return ret[0]

    @xarm_is_connected(_type='set')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='set', default=None)
    def _set_modbus_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_modbus_set_posspd(speed)
        _, err = self._get_modbus_gripper_err_code()
        logger.info('API -> set_modbus_gripper_speed -> ret={}, speed={}, ret2={}, err={}'.format(ret[0], speed, _, err))
        ret[0] = self._check_modbus_code(ret)
        if ret[0] == 0 and self.gripper_error_code == 0:
            self.gripper_speed = speed
        return ret[0]

    @xarm_is_connected(_type='get')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='get', default=None)
    def _get_modbus_gripper_position(self):
        ret = self.arm_cmd.gripper_modbus_get_pos()
        ret[0] = self._check_modbus_code(ret)
        _, err = self._get_modbus_gripper_err_code()
        if ret[0] != 0 or len(ret) <= 1:
            return ret[0], None
        elif _ == 0 and err == 0:
            return ret[0], int(ret[1])
        else:
            return ret[0], None
            # return _ if err == 0 else XCONF.UxbusState.ERR_CODE, None

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='set', default=None)
    def _set_modbus_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):
        if auto_enable and not self.gripper_is_enabled:
            ret = self.arm_cmd.gripper_modbus_set_en(True)
            ret[0] = self._check_modbus_code(ret)
            if ret[0] == 0:
                self.gripper_is_enabled = True
        if speed is not None and self.gripper_speed != speed:
            ret = self.arm_cmd.gripper_modbus_set_posspd(speed)
            ret[0] = self._check_modbus_code(ret)
            if ret[0] == 0:
                self.gripper_speed = speed
        ret = self.arm_cmd.gripper_modbus_set_pos(pos)
        logger.info('API -> set_modbus_gripper_position -> ret={}, pos={}'.format(ret[0], pos))
        ret[0] = self._check_modbus_code(ret)
        if wait:
            is_add = True
            last_pos = 0
            _, p = self._get_modbus_gripper_position()
            if _ == 0 and p is not None:
                last_pos = int(p)
                if last_pos == pos:
                    return 0
                is_add = True if pos > last_pos else False
            count = 0
            count2 = 0
            if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
                timeout = 10
            expired = time.time() + timeout
            while self.connected and time.time() < expired:
                _, p = self._get_modbus_gripper_position()
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
                    if count >= 8:
                        # print('gripper target: {}, current: {}'.format(pos, cur_pos))
                        break
                else:
                    break
                time.sleep(0.2)
            # print('gripper, pos: {}, last: {}'.format(pos, last_pos))
            return ret[0]
        else:
            _, err = self._get_modbus_gripper_err_code()
            # return ret[0] if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] else _
            return ret[0]

    @xarm_is_connected(_type='get')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='get', default=0)
    def _get_modbus_gripper_err_code(self):
        ret = self.arm_cmd.gripper_modbus_get_errcode()
        logger.info('API -> get_modbus_gripper_err_code -> ret={}'.format(ret))
        ret[0] = self._check_modbus_code(ret)
        if ret[0] == 0:
            if ret[1] < 128:
                self._gripper_error_code = ret[1]
                self.gripper_is_enabled = False
                self.gripper_speed = 0
            # if ret[0] == XCONF.UxbusState.ERR_CODE:
            #     self.get_err_warn_code()
            #     if self.error_code == 19 or self.error_code == 28:
            #         return ret[0], ret[1]
            # ret[0] = 0
            return ret[0], ret[1]
        return ret[0], 0

    @xarm_is_connected(_type='set')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='set', default=None)
    def _clean_modbus_gripper_error(self):
        ret = self.arm_cmd.gripper_modbus_clean_err()
        self._gripper_error_code = 0
        _, err = self._get_modbus_gripper_err_code()
        logger.info('API -> clean_modbus_gripper_error -> ret={}, ret2={}, err={}'.format(ret[0], _, err))
        ret[0] = self._check_modbus_code(ret)
        return ret[0]

    @xarm_is_connected(_type='set')
    @check_modbus_baud(baud=GRIPPER_BAUD, _type='set', default=None)
    def _set_modbus_gripper_zero(self):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        ret = self.arm_cmd.gripper_modbus_set_zero()
        _, err = self._get_modbus_gripper_err_code()
        logger.info('API -> set_modbus_gripper_zero -> ret={}, ret2={}, err={}'.format(ret[0], _, err))
        ret[0] = self._check_modbus_code(ret)
        return ret[0]

    @check_modbus_baud(baud=GRIPPER_BAUD, _type='get', default=-99)
    def __bio_gripper_send_modbus(self, data_frame, min_res_len=0):
        return self.getset_tgpio_modbus_data(data_frame, min_res_len=min_res_len)

    def __check_bio_gripper_finish(self, timeout=5, **kwargs):
        failed_cnt = 0
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        check_detected = kwargs.get('check_detected', False)
        while time.time() < expired:
            _, status = self.get_bio_gripper_status()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0:
                code = code if status == XCONF.BioGripperState.IS_MOTION \
                    else APIState.END_EFFECTOR_HAS_FAULT if status == XCONF.BioGripperState.IS_FAULT \
                    else 0
            else:
                code = APIState.NOT_CONNECTED if _ == APIState.NOT_CONNECTED else APIState.CHECK_FAILED if failed_cnt > 10 else code
            if code != APIState.WAIT_FINISH_TIMEOUT:
                break
            time.sleep(0.1)
        return code

    @xarm_is_connected(_type='set')
    def set_bio_gripper_enable(self, enable):
        data_frame = [0x08, 0x06, 0x01, 0x00, 0x00, int(enable)]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        logger.info('API -> set_bio_gripper_enable ->code={}, enable={}'.format(code, enable))
        self.bio_gripper_is_enabled = True if code == 0 else self.bio_gripper_is_enabled
        return code

    @xarm_is_connected(_type='set')
    def set_bio_gripper_speed(self, speed):
        data_frame = [0x08, 0x06, 0x03, 0x03, speed // 256 % 256, speed % 256]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        logger.info('API -> set_bio_gripper_speed ->code={}, speed={}'.format(code, speed))
        self.bio_gripper_speed = speed if code == 0 else self.bio_gripper_speed
        return code

    # @xarm_is_connected(_type='set')
    # def set_bio_gripper_position(self, pos, wait=True, timeout=5):
    #     data_frame = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x80 if pos < 0 else 0x00, 0x00, pos // 256 % 256, pos % 256]
    #     code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
    #     if code == 0 and wait:
    #         code = self.__check_bio_gripper_finish(timeout=timeout)
    #     logger.info('API -> set_bio_gripper_position ->code={}, pos={}'.format(code, pos))
    #     return code

    @xarm_is_connected(_type='set')
    def open_bio_gripper(self, speed=0, wait=True, timeout=5, **kwargs):
        if kwargs.get('auto_enable', False) and not self.bio_gripper_is_enabled:
            self.set_bio_gripper_enable(True)
            time.sleep(2)
        if speed > 0 and speed != self.bio_gripper_speed:
            self.set_bio_gripper_speed(speed)
        # return self.set_bio_gripper_position(-100, wait=wait, timeout=timeout)
        data_frame = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x80, 0x00, 0x00, 0x64]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        if code == 0 and wait:
            code = self.__check_bio_gripper_finish(timeout=timeout, **kwargs)
        logger.info('API -> open_bio_gripper ->code={}'.format(code))
        return code

    @xarm_is_connected(_type='set')
    def close_bio_gripper(self, speed=0, wait=True, timeout=5, **kwargs):
        if kwargs.get('auto_enable', False):
            self.set_bio_gripper_enable(True)
            time.sleep(2)
        if speed > 0 and speed != self.bio_gripper_speed:
            self.set_bio_gripper_speed(speed)
        # return self.set_bio_gripper_position(100, wait=wait, timeout=timeout)
        data_frame = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x64]
        code, _ = self.__bio_gripper_send_modbus(data_frame, 6)
        if code == 0 and wait:
            code = self.__check_bio_gripper_finish(timeout=timeout, **kwargs)
        logger.info('API -> close_bio_gripper ->code={}'.format(code))
        return code

    @xarm_is_connected(_type='get')
    def __get_bio_gripper_register(self, address=0x00, number_of_registers=1):
        data_frame = [0x08, 0x03, 0x00, address, 0x00, number_of_registers]
        return self.__bio_gripper_send_modbus(data_frame, 3 + 2 * number_of_registers)

    @xarm_is_connected(_type='get')
    def get_bio_gripper_status(self):
        code, ret = self.__get_bio_gripper_register(address=0x00)
        status = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        if code == 0:
            if status == XCONF.BioGripperState.IS_FAULT:
                self.get_bio_gripper_error()
            else:
                self.bio_gripper_error_code = 0
        # print('code={}, status={}'.format(code, status))
        return code, status

    @xarm_is_connected(_type='get')
    def get_bio_gripper_error(self):
        code, ret = self.__get_bio_gripper_register(address=0x0F)
        error_code = (ret[-2] * 256 + ret[-1]) if code == 0 else -1
        if code == 0:
            self.bio_gripper_error_code = error_code
        return code, error_code
