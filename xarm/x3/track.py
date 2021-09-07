# -*- coding: utf-8 -*-
# __author: rock
# @time: 2021-07-19

import time
import struct
from .utils import xarm_is_connected, xarm_is_pause, check_modbus_baud, xarm_is_not_simulation_mode
from ..core.config.x_config import XCONF
from ..core.utils.log import logger
from ..core.utils import convert
from .code import APIState
from .gpio import GPIO

TRACK_BAUD = 2000000


class Track(GPIO):

    def __init__(self):
        super(Track, self).__init__()
        self._line_track_error_code = 0
        self.track_is_stop = False

    @property
    def line_track_error_code(self):
        return self._line_track_error_code

    @line_track_error_code.setter
    def line_track_error_code(self, val):
        self._line_track_error_code = val

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_enable(self, enable):
        value = convert.u16_to_bytes(int(enable))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.CON_EN, value, 1)
        _, err = self._get_track_err_code()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)

        if ret[0] == 0 and self._line_track_error_code == 0 and enable:
            self.track_is_enabled = True
        else:
            self.track_is_enabled = False
        self.log_api_info('API -> set_line_track_enable(enable={}) -> code={}'.format(enable, ret[0]), code=ret[0])

        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_back_origin(self, wait=True, auto_enable=True):
        code = self.checkset_modbus_baud(TRACK_BAUD)
        if code != 0:
            return code
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.BACK_ORIGIN, 1, 0x06)
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        self.log_api_info('API -> set_line_track_back_origin() -> code={}'.format(ret[0]), code=ret[0])
        _, err = self._get_track_err_code()
        if ret[0] == 0 and wait:
            return self.__wait_track_back_origin()
        if auto_enable and not self.track_is_enabled:
            code = self.set_line_track_enable(auto_enable)
            if code == 0:
                self.gripper_is_enabled = True
        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_pos(self, pos, wait=True, speed=None, auto_enable=False, timeout=None, **kwargs):
        code = self.checkset_modbus_baud(TRACK_BAUD)
        if code != 0:
            return code
        if auto_enable and not self.track_is_enabled:
            code = self.set_line_track_enable(auto_enable)
            if code == 0:
                self.gripper_is_enabled = True
        if speed is not None and self.line_track_speed != speed:
            code = self.set_line_track_speed(speed)
            if code == 0:
                self.line_track_speed = speed
        value = self.__parse_pos(pos*2000)
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.TAGET_POS, value, 2)
        self.log_api_info('API -> set_track_pos(pos={}) -> code={}'.format(pos, ret[0]), code=ret[0])
        _, err = self._get_track_err_code()
        if self._line_track_error_code != 0:
            print('line track ErrorCode: {}'.format(self._line_track_error_code))
            return APIState.END_EFFECTOR_HAS_FAULT

        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if ret[0] == 0 and wait:
            return self.__check_track_status()

        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_speed(self, speed):
        value = convert.u16_to_bytes(int(speed))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.POS_SPD, value, 1)
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if ret[0] == 0 and self._line_track_error_code == 0:
            self.line_track_speed = speed
        self.log_api_info('API -> set_line_track_speed(speed={}) -> code={}'.format(speed, ret[0]), code=ret[0])
        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def check_line_track_on_zero(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.CHECK_ON_ORIGIN, 1)
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        code = ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT
        self.log_api_info('API -> check_line_track_on_zero() -> code={}'.format(ret[0]), code=ret[0])
        if code == 0 and ret[-1] == 1:
            return code, ret[-1]
        else:
            self._line_track_error_code = 8
            return code, 0

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_track_status(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.GET_STATUS, 1)
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        status = 0
        if ret[0] == 0 and len(ret) == 7:
            status = convert.bytes_to_u16(ret[5:7])
        return ret[0], status

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def _get_track_err_code(self):
        ret = self.__get_track_err_code()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if ret[0] == 0:
            if ret[1] < 128:
                self._line_track_error_code = ret[1]
                if self._line_track_error_code != 0:
                    self.track_is_enabled = False
                    self.line_track_speed = 0
            return ret[0], ret[1]
        return ret[0], 0

    @xarm_is_connected(_type='set')
    def clean_line_track_err(self):
        value = convert.u16_to_bytes(int(1))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.RESET_ERR, value, 1)
        self._line_track_error_code = 0
        _, err = self._get_track_err_code()
        self.log_api_info('API -> clean_line_track_err -> code={}, code2={}, err={}'.format(ret[0], _, err), code=ret[0])
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    def __check_track_status(self, timeout=100):
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 100
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.time() < expired:
            _, status = self.get_track_status()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0 and (status & 0x03 == 0 or status & 0x03 == 2):
                return 0
            else:
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.1)
        return code

    def __get_track_err_code(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.ERR_CODE, 1)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) == 7:
            ret1[1] = convert.bytes_to_u16(ret[5:7])
        else:
            if ret1[0] == 0:
                ret1[0] = XCONF.UxbusState.ERR_LENG
        return ret1

    @staticmethod
    def __parse_pos(pos):
        value = bytes([(int(pos) >> 24) & 0xFF])
        value += bytes([(int(pos) >> 16) & 0xFF])
        value += bytes([(int(pos) >> 8) & 0xFF])
        value += bytes([int(pos) & 0xFF])
        return value

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_line_track_pos(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.CURR_POS, 2)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) == 9:
            ret1[1] = convert.bytes_to_long_big(ret[5:9])
        else:
            if ret1[0] == 0:
                ret1[0] = XCONF.UxbusState.ERR_LENG
        ret1[0] = self._check_modbus_code(ret1, only_check_code=True)
        _, err = self._get_track_err_code()
        if self.line_track_error_code != 0:
            return APIState.END_EFFECTOR_HAS_FAULT
        if ret1[0] != 0 or len(ret1) <= 1:
            return ret1[0], None
        elif _ == 0 and err == 0:
            return ret1[0], int(int(ret1[1])/2000)
        else:
            return ret1[0], None

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def stop_line_track(self):
        value = convert.u16_to_bytes(int(1))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.STOP_TRACK, value, 1)
        # _, err = self._get_track_err_code()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        re = self.get_track_status()
        if ret[0] == 0:
            self.track_is_stop = True
        else:
            self.track_is_stop = False
        self.log_api_info('API -> stop_line_track() -> code={}'.format(ret[0]), code=ret[0])

        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    def __wait_track_back_origin(self, timeout=10):
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 10
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.time() < expired:
            _, status = self.check_line_track_on_zero()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0 and status == 1:
                return 0
            else:
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.1)
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def get_line_track_version(self):
        code = self.checkset_modbus_baud(TRACK_BAUD)
        if code != 0:
            return code, '*.*.*'
        versions = ['*', '*', '*']
        ret1 = self.arm_cmd.track_modbus_r16s(0x0801, 1)
        ret2 = self.arm_cmd.track_modbus_r16s(0x0802, 1)
        ret3 = self.arm_cmd.track_modbus_r16s(0x0803, 1)
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

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def get_line_track_sn(self):
        rd_sn = ''
        ret = [0, '']
        for i in range(0, 14):
            ret = self.arm_cmd.track_modbus_r16s(0x0B10 + i, 1)
            time.sleep(0.05)
            rd_sn = ''.join([rd_sn, chr(ret[-1])])
            # if ret[0] != 0:
            #     return ret[0], ''
        return ret[0], rd_sn

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_sn(self, sn):
        code = 0
        if len(sn) == 14:
            for i in range(0, 14):
                value = convert.u16_to_bytes(ord(sn[i]))
                ret = self.arm_cmd.track_modbus_w16s(0x1B10 + i, value, 1)
                time.sleep(0.05)
                if ret[0] != 0:
                    return 1
                code = ret[0]
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def get_line_track_enable(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.CON_EN, 1)
        _, err = self._get_track_err_code()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)

        if ret[0] == 0 and ret[-1] == 0 and self._line_track_error_code == 0:
            self.track_is_enabled = False

        self.log_api_info('API -> get_line_track_enable() -> code={}'.format(ret[0]), code=ret[0])

        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT, ret[-1]
