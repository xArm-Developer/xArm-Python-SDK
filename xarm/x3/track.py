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
        super(GPIO, self).__init__()
        self._line_track_error_code = 0

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

        if ret[0] == 0 and self.line_track_error_code == 0:
            self.track_is_enabled = True
        self.log_api_info('API -> set_line_track_enable(enable={}) -> code={}'.format(enable, ret[0]), code=ret[0])

        return ret[0] if self._line_track_error_code == 0 else APIState.END_EFFECTOR_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_back_origin(self, wait=True, auto_enable=True):
        ret = [0]
        if wait:
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move()
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0
        code = self.checkset_modbus_baud(TRACK_BAUD)
        if code != 0:
            return code

        st = time.time()
        while wait:
            ret = self.arm_cmd.track_modbus_r16s(0x0A0A, 1, 0x06)
            ret[0] = self._check_modbus_code(ret, only_check_code=True)
            time.sleep(0.2)
            code, status = self.check_line_track_on_zero()
            time.sleep(0.2)
            et = time.time()
            if code == 0 and status == 1:
                break
            if et - st > 15:
                return APIState.WAIT_FINISH_TIMEOUT

        if auto_enable:
            self.set_line_track_enable(auto_enable)
        self.log_api_info('API -> line_track_back_zero() -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_pos(self, pos, wait=True, speed=None, auto_enable=False, timeout=None, **kwargs):
        if wait:
            has_error = self.error_code != 0
            is_stop = self.is_stop
            code = self.wait_move()
            if not (code == 0 or (is_stop and code == APIState.EMERGENCY_STOP)
                    or (has_error and code == APIState.HAS_ERROR)):
                return code
        if self.check_is_simulation_robot():
            return 0
        code = self.checkset_modbus_baud(TRACK_BAUD)
        if code != 0:
            return code
        if auto_enable and not self.track_is_enabled:
            self.set_line_track_enable(auto_enable)
            time.sleep(0.2)
        if speed is not None:
            self.set_line_track_speed(speed)
        value = self.__parse_pos(pos*2000)
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.TAGET_POS, value, 2)
        if ret[0] == 0 and wait:
            self.__check_track_status()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        self.log_api_info('API -> set_track_pos(pos={}) -> code={}'.format(pos, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def set_line_track_speed(self, speed):
        ret = [0]
        if speed is not None and speed != self.last_used_track_speed:
            value = convert.u16_to_bytes(int(speed))
            ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.POS_SPD, value, 1)
            ret[0] = self._check_modbus_code(ret, only_check_code=True)
            if ret[0] == 0:
                self.last_used_track_speed = speed
        self.log_api_info('API -> set_line_track_speed(speed={}) -> code={}'.format(speed, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None)
    def check_line_track_on_zero(self):
        ret = self.arm_cmd.track_modbus_r16s(0x004F, 1)
        _, err = self._get_track_err_code()
        ret[0] = self._check_modbus_code(ret, only_check_code=True)
        if ret[0] == 0 and ret[-1] == 1 and err == 0:
            self.log_api_info('API -> check_line_track_on_zero() -> code={}'.format(ret[0]), code=ret[0])
            return ret[0], ret[-1]
        return ret[0], 0

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    def get_track_status(self):
        ret = self.arm_cmd.track_modbus_r16s(0x0000, 1)
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
                    self.last_used_track_speed = 0
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

    def __check_track_status(self, timeout=10):
        start_move = False
        not_start_move_cnt = 0
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 10
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.time() < expired:
            _, status = self._get_track_err_code()
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
