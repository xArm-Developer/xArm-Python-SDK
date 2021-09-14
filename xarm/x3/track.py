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
        self._linear_track_error_code = 0
        self._linear_track_is_stop = False

    @property
    def linear_track_error_code(self):
        return self._linear_track_error_code

    @linear_track_error_code.setter
    def linear_track_error_code(self, val):
        self._linear_track_error_code = val

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_enable(self, enable):
        value = convert.u16_to_bytes(int(enable))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.CON_EN, value, 1)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        if ret[0] == 0 and self._linear_track_error_code == 0 and enable:
            self.linear_track_is_enabled = True
        else:
            self.linear_track_is_enabled = False
        self.get_linear_track_enable()
        self.log_api_info('API -> set_linear_track_enable(enable={}) -> code={}, err={}'.format(enable, ret[0], err), code=ret[0])
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_back_origin(self, wait=True, **kwargs):
        auto_enable = kwargs.get('auto_enable', True)
        timeout = kwargs.get('timeout', 10)
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.BACK_ORIGIN, 1, 0x06)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        self.log_api_info('API -> set_linear_track_back_origin() -> code={}, err={}'.format(ret[0], err), code=ret[0])
        if ret[0] == 0 and wait:
            ret[0] = self.__wait_linear_track_back_origin(timeout)
        if auto_enable:
            ret[0] = self.set_linear_track_enable(True)
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_pos(self, pos, speed=None, wait=True, timeout=100, **kwargs):
        auto_enable = kwargs.get('auto_enable', False)
        code, status = self.check_linear_track_on_zero()
        code, is_enable = self.get_linear_track_enable()
        if code == 0 and (status == 0 or is_enable == 0):
            return APIState.LINEAR_TRACK_NOT_INIT
        if auto_enable and not self.linear_track_is_enabled:
            self.set_linear_track_enable(auto_enable)
        if speed is not None and self.linear_track_speed != speed:
            self.set_linear_track_speed(speed)

        value = convert.int32_to_bytes(int(pos * 2000), is_big_endian=True)
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.TAGET_POS, value, 2)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        self.log_api_info('API -> set_linear_track_pos(pos={}) -> code={}, err={}'.format(pos, ret[0], err), code=ret[0])

        if ret[0] == 0 and wait:
            return self.__wait_linear_track_stop(timeout)
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_speed(self, speed):
        value = convert.u16_to_bytes(int(speed*6.667))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.POS_SPD, value, 1)
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        _, err = self.get_linear_track_error()
        if ret[0] == 0 and self._linear_track_error_code == 0:
            self.linear_track_speed = speed
        self.log_api_info('API -> set_linear_track_speed(speed={}) -> code={}, err={}'.format(speed, ret[0], err), code=ret[0])
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def check_linear_track_on_zero(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.CHECK_ON_ORIGIN, 1)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
        code = ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT
        # self.log_api_info('API -> check_linear_track_on_zero() -> code={}, err={}'.format(ret[0], err), code=ret[0])
        return code, ret[-1] if code == 0 else 0

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=-99, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def get_linear_track_status(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.GET_STATUS, 1)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
        code = ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT
        return code, convert.bytes_to_u16(ret[5:7]) if ret[0] == 0 else 0

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=-99, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def get_linear_track_error(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.ERR_CODE, 1)
        # ret = self.arm_cmd.track_modbus_r16s(0x0A00, 1)
        ret[0] = self._check_modbus_code(ret, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
        if ret[0] == 0:
            self._linear_track_error_code = convert.bytes_to_u16(ret[5:7])
        return ret[0], self._linear_track_error_code

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def clean_linear_track_error(self):
        value = convert.u16_to_bytes(int(1))
        self._linear_track_error_code = 0
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.RESET_ERR, value, 1)
        _, err = self.get_linear_track_error()
        self.log_api_info('API -> clean_linear_track_error -> code={}, code2={}, err={}'.format(ret[0], _, err), code=ret[0])
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    def __wait_linear_track_stop(self, timeout=100):
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 100
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.time() < expired:
            _, status = self.get_linear_track_status()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0 and (status & 0x01 == 0):
                return 0
            else:
                if self._linear_track_error_code != 0:
                    return APIState.LINEAR_TRACK_HAS_FAULT
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.1)
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def get_linear_track_pos(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.CURR_POS, 2)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=9, host_id=XCONF.LINEER_TRACK_HOST_ID)
        if ret[0] == 0 and err == 0:
            return ret[0], convert.bytes_to_long_big(ret[5:9]) / 2000
        elif err != 0:
            return APIState.LINEAR_TRACK_HAS_FAULT, 0
        else:
            return ret[0], 0

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def stop_linear_track(self):
        value = convert.u16_to_bytes(int(1))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.STOP_TRACK, value, 1)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        self._linear_track_is_stop = True if ret[0] == 0 else False
        self.log_api_info('API -> stop_linear_track() -> code={}, err={}'.format(ret[0], err), code=ret[0])

        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    def __wait_linear_track_back_origin(self, timeout=10):
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 10
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.time() < expired:
            _, status = self.check_linear_track_on_zero()
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0 and status == 1:
                return 0
            else:
                if self._linear_track_error_code != 0:
                    return APIState.LINEAR_TRACK_HAS_FAULT
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.1)
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, '*.*.*'))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default='*.*.*', host_id=XCONF.LINEER_TRACK_HOST_ID)
    def get_linear_track_version(self):
        versions = ['*', '*', '*']
        ret1 = self.arm_cmd.track_modbus_r16s(0x0801, 1)
        ret2 = self.arm_cmd.track_modbus_r16s(0x0802, 1)
        ret3 = self.arm_cmd.track_modbus_r16s(0x0803, 1)
        ret1[0] = self._check_modbus_code(ret1, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
        ret2[0] = self._check_modbus_code(ret2, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
        ret3[0] = self._check_modbus_code(ret3, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)

        code = 0

        if ret1[0] == 0:
            versions[0] = convert.bytes_to_u16(ret1[5:7])
        else:
            code = ret1[0]

        if ret2[0] == 0:
            versions[1] = convert.bytes_to_u16(ret2[5:7])
        else:
            code = ret2[0]

        if ret3[0] == 0:
            versions[2] = convert.bytes_to_u16(ret3[5:7])
        else:
            code = ret3[0]

        return code, '.'.join(map(str, versions))

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, None))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def get_linear_track_sn(self):
        rd_sn = ''
        ret = [0, '']
        for i in range(0, 14):
            ret = self.arm_cmd.track_modbus_r16s(0x0B10 + i, 1)
            ret[0] = self._check_modbus_code(ret, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
            if ret[0] == 0:
                rd_sn = ''.join([rd_sn, chr(ret[-1])])
            else:
                rd_sn = ''.join([rd_sn, '*'])
            time.sleep(0.05)
        return ret[0], rd_sn

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_sn(self, sn):
        assert len(sn) >= 14, 'The length of SN is wrong'
        code = 0
        if len(sn) == 14:
            for i in range(0, 14):
                value = convert.u16_to_bytes(ord(sn[i]))
                ret = self.arm_cmd.track_modbus_w16s(0x1B10 + i, value, 1)
                code = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
                if code != 0:
                    break
                time.sleep(0.05)
        return code

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def get_linear_track_enable(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.CON_EN, 1)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)

        if ret[0] == 0 and self._linear_track_error_code == 0:
            self.linear_track_is_enabled = ret[-1] != 0
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT, ret[-1]

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, 0))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=None)
    def get_linear_track_sci(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.GET_TRACK_SCI, 1)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
        if ret[0] == 0 and self._linear_track_error_code == 0:
            self.linear_track_is_enabled = ret[-1] != 0
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT, ret[-1]

    def get_linear_track_sco(self):
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.GET_TRACK_SCO, 1)
        _, err = self.get_linear_track_error()
        ret[0] = self._check_modbus_code(ret, length=7, host_id=XCONF.LINEER_TRACK_HOST_ID)
        if ret[0] == 0 and self._linear_track_error_code == 0:
            self.linear_track_is_enabled = ret[-1] != 0
        return ret[0] if self._linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT, ret[-1]

