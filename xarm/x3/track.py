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
        self._linear_track_status = {
            'pos': 0,
            'status': 0,
            'error': 0,
            'is_enabled': 0,
            'on_zero': 0,
            'sci': 1,
            'sco': [0, 0],
        }

    @property
    def linear_track_status(self):
        return self._linear_track_status

    @property
    def linear_track_error_code(self):
        return self._linear_track_status['error']

    @linear_track_error_code.setter
    def linear_track_error_code(self, val):
        self._linear_track_status['error'] = val

    @xarm_is_connected(_type='get')
    @xarm_is_not_simulation_mode(ret=(0, []))
    @check_modbus_baud(baud=TRACK_BAUD, _type='get', default=-99, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def _get_linear_track_registers(self, addr, number_of_registers=1):
        ret = self.arm_cmd.track_modbus_r16s(addr, number_of_registers)
        ret[0] = self._check_modbus_code(ret, length=5 + 2 * number_of_registers, host_id=XCONF.LINEER_TRACK_HOST_ID)
        return ret[0], ret[1:]

    def get_linear_track_registers(self, addr=0x0A20, number_of_registers=8):
        assert (addr == 0x0A20 and number_of_registers >= 2) \
            or (0x0A22 <= addr <= 0x0A27 and number_of_registers >= 1), \
            'parameters error, only support (addr == 0x0A20 and number_of_registers >= 2) ' \
            'or (0x0A22 <= addr <= 0x0A27 and number_of_registers >= 1)'
        code, data = self._get_linear_track_registers(addr, number_of_registers=number_of_registers)
        if code == 0:
            if addr == 0x0A20 and number_of_registers >= 2:
                self._linear_track_status['pos'] = round(convert.bytes_to_long_big(data[4:8]) / 2000)
            if 0x0A22 - number_of_registers < addr <= 0x0A22:
                start_inx = (0x0A22 - addr) * 2 + 4
                self._linear_track_status['status'] = convert.bytes_to_u16(data[start_inx:start_inx+2])
            if 0x0A23 - number_of_registers < addr <= 0x0A23:
                start_inx = (0x0A23 - addr) * 2 + 4
                self._linear_track_status['error'] = convert.bytes_to_u16(data[start_inx:start_inx+2])
            if 0x0A24 - number_of_registers < addr <= 0x0A24:
                start_inx = (0x0A24 - addr) * 2 + 4
                self._linear_track_status['is_enabled'] = convert.bytes_to_u16(data[start_inx:start_inx+2]) & 0x01
                self.linear_track_is_enabled = self._linear_track_status['is_enabled'] == 1
            if 0x0A25 - number_of_registers < addr <= 0x0A25:
                start_inx = (0x0A25 - addr) * 2 + 4
                self._linear_track_status['on_zero'] = convert.bytes_to_u16(data[start_inx:start_inx+2]) & 0x01
            if 0x0A26 - number_of_registers < addr <= 0x0A26:
                start_inx = (0x0A26 - addr) * 2 + 4
                self._linear_track_status['sci'] = convert.bytes_to_u16(data[start_inx:start_inx+2]) >> 1 & 0x01
            if 0x0A27 - number_of_registers < addr <= 0x0A27:
                start_inx = (0x0A27 - addr) * 2 + 4
                sco = convert.bytes_to_u16(data[start_inx:start_inx+2])
                self._linear_track_status['sco'][0] = sco & 0x01
                self._linear_track_status['sco'][1] = sco >> 1 & 0x01
        return code, self._linear_track_status

    def get_linear_track_pos(self):
        code, _ = self.get_linear_track_registers(addr=0x0A20, number_of_registers=2)
        return code, self._linear_track_status['pos']

    def get_linear_track_status(self):
        code, _ = self.get_linear_track_registers(addr=0x0A22, number_of_registers=1)
        return code, self._linear_track_status['status']

    def get_linear_track_error(self):
        code, _ = self.get_linear_track_registers(addr=0x0A23, number_of_registers=1)
        return code, self._linear_track_status['error']

    def get_linear_track_is_enabled(self):
        code, _ = self.get_linear_track_registers(addr=0x0A24, number_of_registers=1)
        return code, self._linear_track_status['is_enabled']

    def get_linear_track_on_zero(self):
        code, _ = self.get_linear_track_registers(addr=0x0A25, number_of_registers=1)
        return code, self._linear_track_status['on_zero']

    def get_linear_track_sci(self):
        code, _ = self.get_linear_track_registers(addr=0x0A26, number_of_registers=1)
        return code, self._linear_track_status['sci']

    def get_linear_track_sco(self):
        code, _ = self.get_linear_track_registers(addr=0x0A27, number_of_registers=1)
        return code, self._linear_track_status['sco']

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_enable(self, enable):
        value = convert.u16_to_bytes(int(enable))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.CON_EN, value, 1)
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        # get_status: error, is_enable, on_zero
        code2, status = self.get_linear_track_registers(addr=0x0A23, number_of_registers=3)
        if ret[0] == 0 and self._linear_track_status['error'] == 0 and enable:
            self.linear_track_is_enabled = self._linear_track_status['is_enabled'] == 1
        else:
            self.linear_track_is_enabled = False
        self.log_api_info('API -> set_linear_track_enable(enable={}) -> code1={}, code2={}, err={}, enabled={}, zero={}'.format(
            enable, ret[0], code2, status['error'], status['is_enabled'], status['on_zero']), code=ret[0])
        return ret[0] if self.linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_back_origin(self, wait=True, **kwargs):
        auto_enable = kwargs.get('auto_enable', True)
        timeout = kwargs.get('timeout', 10)
        ret = self.arm_cmd.track_modbus_r16s(XCONF.ServoConf.BACK_ORIGIN, 1, 0x06)
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        # get_status: error, is_enable, on_zero
        code2, status = self.get_linear_track_registers(addr=0x0A23, number_of_registers=3)
        self.log_api_info(
            'API -> set_linear_track_back_origin() -> code1={}, code2={}, err={}, enabled={}, zero={}'.format(
                ret[0], code2, status['error'], status['is_enabled'], status['on_zero']), code=ret[0])
        if ret[0] == 0 and wait:
            ret[0] = self.__wait_linear_track_back_origin(timeout)
        if auto_enable:
            ret[0] = self.set_linear_track_enable(True)
        return ret[0] if self.linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_pos(self, pos, speed=None, wait=True, timeout=100, **kwargs):
        auto_enable = kwargs.get('auto_enable', True)
        # get_status: error, is_enable, on_zero
        code, status = self.get_linear_track_registers(addr=0x0A23, number_of_registers=3)
        if code == 0 and status['on_zero'] != 1:
            logger.warn('linear track is not on zero, please set linear track back to origin')
            return APIState.LINEAR_TRACK_NOT_INIT
        if auto_enable and (code != 0 or status['is_enabled'] != 1):
            self.set_linear_track_enable(auto_enable)
        if speed is not None and self.linear_track_speed != speed:
            self.set_linear_track_speed(speed)
        value = convert.int32_to_bytes(int(pos * 2000), is_big_endian=True)
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.TAGET_POS, value, 2)
        self.get_linear_track_registers(addr=0x0A23, number_of_registers=3)
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        self.log_api_info('API -> set_linear_track_pos(pos={}) -> code={}, err={}, enabled={}, zero={}'.format(
            pos, ret[0], self._linear_track_status['error'],
            self._linear_track_status['is_enabled'], self._linear_track_status['on_zero']), code=ret[0])
        if ret[0] == 0 and wait:
            return self.__wait_linear_track_stop(timeout)
        return ret[0] if self.linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_speed(self, speed):
        value = convert.u16_to_bytes(int(speed*6.667))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.POS_SPD, value, 1)
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        if ret[0] == 0:
            self.linear_track_speed = speed
        self.log_api_info('API -> set_linear_track_speed(speed={}) -> code={}'.format(speed, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def set_linear_track_stop(self):
        value = convert.u16_to_bytes(int(1))
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.STOP_TRACK, value, 1)
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        # get_status: error, is_enable, on_zero
        code2, status = self.get_linear_track_registers(addr=0x0A22, number_of_registers=2)
        self.log_api_info('API -> set_linear_track_stop() -> code={}, code2={}, status={}, err={}'.format(
            ret[0], code2, status['status'], status['error']), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    @check_modbus_baud(baud=TRACK_BAUD, _type='set', default=None, host_id=XCONF.LINEER_TRACK_HOST_ID)
    def clean_linear_track_error(self):
        value = convert.u16_to_bytes(int(1))
        self.linear_track_error_code = 0
        ret = self.arm_cmd.track_modbus_w16s(XCONF.ServoConf.RESET_ERR, value, 1)
        _, err = self.get_linear_track_error()
        self.log_api_info('API -> clean_linear_track_error -> code={}, code2={}, err={}'.format(ret[0], _, err),
                          code=ret[0])
        ret[0] = self._check_modbus_code(ret, length=8, host_id=XCONF.LINEER_TRACK_HOST_ID)
        return ret[0] if self.linear_track_error_code == 0 else APIState.LINEAR_TRACK_HAS_FAULT

    def __wait_linear_track_stop(self, timeout=100):
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 100
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.time() < expired:
            _, status = self.get_linear_track_registers(addr=0x0A22, number_of_registers=5)
            if _ == 0 and status['sci'] == 0:
                return APIState.LINEAR_TRACK_SCI_IS_LOW
            if _ == 0 and status['error'] != 0:
                return APIState.LINEAR_TRACK_HAS_FAULT
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0 and (status['status'] & 0x01 == 0):
                return 0
            else:
                if failed_cnt > 10:
                    return APIState.CHECK_FAILED
            time.sleep(0.1)
        return code

    def __wait_linear_track_back_origin(self, timeout=10):
        failed_cnt = 0
        if not timeout or not isinstance(timeout, (int, float)) or timeout <= 0:
            timeout = 10
        expired = time.time() + timeout
        code = APIState.WAIT_FINISH_TIMEOUT
        while self.connected and time.time() < expired:
            _, status = self.get_linear_track_registers(addr=0x0A22, number_of_registers=5)
            if _ == 0 and status['sci'] == 0:
                return APIState.LINEAR_TRACK_SCI_IS_LOW
            if _ == 0 and status['error'] != 0:
                return APIState.LINEAR_TRACK_HAS_FAULT
            failed_cnt = 0 if _ == 0 else failed_cnt + 1
            if _ == 0 and status['on_zero'] == 1:
                return 0
            else:
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
