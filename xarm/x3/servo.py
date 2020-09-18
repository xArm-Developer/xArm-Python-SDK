#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from .utils import xarm_is_connected
from ..core.config.x_config import XCONF
from ..core.config.x_code import ServoError
from ..core.utils.log import logger, pretty_print
from .base import Base


class Servo(Base):
    def __init__(self):
        super(Servo, self).__init__()

    @xarm_is_connected(_type='get')
    def get_servo_debug_msg(self, show=False, lang='en'):
        ret = self.arm_cmd.servo_get_dbmsg()
        dbmsg = []
        lang = lang if lang == 'cn' else 'en'
        if self._check_code(ret[0]) == 0:
            for i in range(1, 9):
                servo_error = ServoError(ret[i * 2], status=ret[i * 2 - 1])
                name = ('伺服-{}'.format(i) if lang == 'cn' else 'Servo-{}'.format(i)) if i < 8 else ('机械爪' if lang == 'cn' else 'Gripper')
                dbmsg.append({
                    'name': name,
                    'servo_id': i,
                    'status': servo_error.status,
                    'code': servo_error.code,
                    'title': servo_error.title[lang],
                    'desc': servo_error.description[lang]
                })
        if show:
            pretty_print('************* {}, {}: {} **************'.format(
                '获取伺服信息' if lang == 'cn' else 'GetServoDebugMsg',
                '状态' if lang == 'cn' else 'Status',
                ret[0]), color='light_blue')
            for servo_info in dbmsg:
                color = 'red' if servo_info['code'] != 0 or servo_info['status'] != 0 else 'white'
                pretty_print('* {}, {}: {}, {}: {}, {}: {}'.format(
                    servo_info['name'],
                    '状态' if lang == 'cn' else 'Status',
                    servo_info['status'],
                    '错误码' if lang == 'cn' else 'Code',
                    servo_info['code'],
                    '信息' if lang == 'cn' else 'Info',
                    servo_info['title']), color=color)
            pretty_print('*' * 50, color='light_blue')
        return ret[0], dbmsg

    @xarm_is_connected(_type='set')
    def set_servo_zero(self, servo_id=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8, 'The value of parameter servo_id can only be 1-8.'
        ret = self.arm_cmd.servo_set_zero(servo_id)
        self.log_api_info('API -> set_servo_zero(servo_id={}) -> code={}'.format(servo_id, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_servo_addr_16(self, servo_id=None, addr=None, value=None, id_check=True):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :param id_check:
        :return: 
        """
        if id_check:
            assert isinstance(servo_id, int) and 1 <= servo_id <= 7, 'The value of parameter servo_id can only be 1-7.'
        assert addr is not None, 'The value of parameter addr cannot be None.'
        assert value is not None, 'The value of parameter value cannot be None.'
        ret = self.arm_cmd.servo_addr_w16(servo_id, addr, value)
        self.log_api_info('API -> set_servo_addr_16(servo_id={}, addr={}, value={}) -> code={}'.format(servo_id, addr, value, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_addr_16(self, servo_id=None, addr=None, id_check=True):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
        if id_check:
            assert isinstance(servo_id, int) and 1 <= servo_id <= 7, 'The value of parameter servo_id can only be 1-7.'
        assert addr is not None, 'The value of parameter addr cannot be None.'
        ret = self.arm_cmd.servo_addr_r16(servo_id, addr)
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_servo_addr_32(self, servo_id=None, addr=None, value=None, id_check=True):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :return: 
        """
        if id_check:
            assert isinstance(servo_id, int) and 1 <= servo_id <= 7, 'The value of parameter servo_id can only be 1-7.'
        assert addr is not None, 'The value of parameter addr cannot be None.'
        assert value is not None, 'The value of parameter value cannot be None.'
        ret = self.arm_cmd.servo_addr_w32(servo_id, addr, value)
        self.log_api_info('API -> set_servo_addr_32(servo_id={}, addr={}, value={}) -> code={}'.format(servo_id, addr, value, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_addr_32(self, servo_id=None, addr=None, id_check=True):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
        if id_check:
            assert isinstance(servo_id, int) and 1 <= servo_id <= 7, 'The value of parameter servo_id can only be 1-7.'
        assert addr is not None, 'The value of parameter addr cannot be None.'
        ret = self.arm_cmd.servo_addr_r32(servo_id, addr)
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def clean_servo_error(self, servo_id=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :return: 
        """
        return self.set_servo_addr_16(servo_id, 0x0109, 1)

    @xarm_is_connected(_type='get')
    def get_servo_state(self, servo_id):
        """
        获取运行状态
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0000)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_rotate_speed(self, servo_id):
        """
        获取转速
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0001)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_current_percentage(self, servo_id):
        """
        获取电流百分比
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0002)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_current(self, servo_id):
        """
        获取电流
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0003)
        return ret[0], ret[1] / 100

    @xarm_is_connected(_type='get')
    def get_servo_command_position(self, servo_id):
        """
        获取指令位置
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_32(servo_id, 0x0004)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_position(self, servo_id):
        """
        获取电机位置
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_32(servo_id, 0x0006)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_position_deviation(self, servo_id):
        """
        获取位置误差
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_32(servo_id, 0x0008)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_electrical_angle(self, servo_id):
        """
        获取电角度
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x000B)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_drv8323_sr0_register(self, servo_id):
        """
        获取DRV8323_SR0状态寄存器
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x000C)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_drv8323_sr1_register(self, servo_id):
        """
        获取DRV8323_SR1状态寄存器
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x000D)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_temperature(self, servo_id):
        """
        获取当前温度
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x000E)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_alarm_code(self, servo_id):
        """
        获取当前报警代码
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x000F)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_alarm_current(self, servo_id):
        """
        获取报警发生时的电流值
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0010)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_alarm_speed(self, servo_id):
        """
        获取报警发生时的速度值
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0011)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_alarm_voltage(self, servo_id):
        """
        获取报警发生时的输入电压值
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0012)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_bus_voltage(self, servo_id):
        """
        获取母线电压
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x0018)
        return ret[0], ret[1] / 100

    @xarm_is_connected(_type='get')
    def get_servo_mu_state(self, servo_id):
        """
        获取MU当前状态
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x001E)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_mu_alarm_count(self, servo_id):
        """
        获取MU上电后报警次数
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_16(servo_id, 0x001F)
        return ret

    @xarm_is_connected(_type='get')
    def get_servo_feedback_position(self, servo_id):
        """
        获取关节反馈位置
        :param servo_id: 
        :return: 
        """
        ret = self.get_servo_addr_32(servo_id, 0x0040)
        return ret

    # @xarm_is_connected(_type='get')
    # def get_servo_current(self, servo_id):
    #     """
    #     获取电流
    #     :param servo_id:
    #     :return:
    #     """
    #     ret = self.get_servo_addr_16(servo_id, 0x0042)
    #     return ret

    @xarm_is_connected(_type='get')
    def get_servo_version(self, servo_id=1):
        """
        获取关节版本
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8, 'The value of parameter servo_id can only be 1-8.'

        def _get_servo_version(id_num):
            versions = ['*', '*', '*']
            ret1 = self.get_servo_addr_16(id_num, 0x0801)
            ret2 = self.get_servo_addr_16(id_num, 0x0802)
            ret3 = self.get_servo_addr_16(id_num, 0x0803)
            code = 0
            if ret1[0] == 0:
                versions[0] = ret1[1]
            else:
                code = ret1[0]
            if ret2[0] == 0:
                versions[1] = ret2[1]
            else:
                code = ret2[0]
            if ret3[0] == 0:
                versions[2] = ret3[1]
            else:
                code = ret3[0]
            # if code != 0:
            #     _, err_warn = self.get_err_warn_code()
            #     if _ in [0, 1, 2]:
            #         if err_warn[0] not in [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 28]:
            #             versions = [ret1[1], ret2[1], ret3[1]]
            return code, '.'.join(map(str, versions))

        if servo_id > self.axis:
            code = 0
            versions = []
            for i in range(1, self.axis + 1):
                ret = _get_servo_version(i)
                if ret[0] != 0:
                    code = ret[0]
                versions.append(ret[1])
            return code, versions
        else:
            return _get_servo_version(servo_id)

    @xarm_is_connected(_type='get')
    def get_harmonic_type(self, servo_id=1):
        """
        获取关节版本
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8, 'The value of parameter servo_id can only be 1-8.'

        if servo_id > self.axis:
            code = 0
            types = []
            for i in range(1, self.axis + 1):
                ret = self.get_servo_addr_16(i, 0x081F)
                if ret[0] != 0:
                    code = ret[0]
                types.append(ret[1])
            return code, types
        else:
            return self.get_servo_addr_16(servo_id, 0x081F)

    @xarm_is_connected(_type='get')
    def get_servo_error_code(self, servo_id=None):
        assert servo_id is None or (isinstance(servo_id, int) and servo_id >= 1), \
            'The value of parameter servo_id must be greater than 1 or None.'
        code = 0
        if servo_id is None or servo_id > self.axis:
            count = 7 if servo_id == 8 else self.axis
            errcodes = [0] * count
            for i in range(count):
                ret = self.get_servo_addr_32(i + 1, XCONF.ServoConf.CURR_POS)
                if ret[0] == XCONF.UxbusState.ERR_CODE:
                    _, err_warn = self.get_err_warn_code()
                    if _ == 0:
                        if i + 11 == err_warn[0]:
                            errcodes[i] = ret[1]
                        else:
                            errcodes[i] = 0
                    else:
                        code = _
                        logger.error('Get controller errwarn: ret={}, errwarn={}'.format(code, err_warn))
                        errcodes[i] = ret[1]
        else:
            errcodes = 0
            ret = self.get_servo_addr_32(servo_id, XCONF.ServoConf.CURR_POS)
            if ret[0] == XCONF.UxbusState.ERR_CODE:
                _, err_warn = self.get_err_warn_code()
                if _ == 0:
                    if servo_id + 10 == err_warn[0]:
                        errcodes = ret[1]
                    else:
                        errcodes = 0
                else:
                    code = _
                    logger.error('Get controller errwarn: ret={}, errwarn={}'.format(code, err_warn))
                    errcodes = ret[1]
        return code, errcodes

    @xarm_is_connected(_type='set')
    def clean_servo_pvl_err(self, servo_id=None):
        assert servo_id is None or (isinstance(servo_id, int) and servo_id >= 1), \
            'The value of parameter servo_id must be greater than 1 or None.'
        if servo_id is None or servo_id > self.axis:
            count = 7 if servo_id == 8 else self.axis
            ids = range(count)
        else:
            ids = [servo_id - 1]
        _, errcode = self.get_servo_error_code()
        for i in ids:
            if errcode[i] == 0x12:
                self.set_servo_addr_16(i + 1, XCONF.ServoConf.RESET_PVL, 0x0002)
                self.set_servo_addr_16(i + 1, XCONF.ServoConf.RESET_ERR, 1)
        return 0

    @xarm_is_connected(_type='get')
    def get_servo_all_pids(self, servo_id=None):
        assert servo_id is None or (isinstance(servo_id, int) and servo_id >= 1), \
            'The value of parameter servo_id must be greater than 1 or None.'
        self.clean_error()
        self.clean_warn()
        addrs = [
            XCONF.ServoConf.POS_KP, XCONF.ServoConf.POS_FWDKP, XCONF.ServoConf.POS_PWDTC,
            XCONF.ServoConf.SPD_KP, XCONF.ServoConf.SPD_KI, XCONF.ServoConf.CURR_KP,
            XCONF.ServoConf.CURR_KI, XCONF.ServoConf.SPD_IFILT, XCONF.ServoConf.SPD_OFILT,
            XCONF.ServoConf.CURR_IFILT, XCONF.ServoConf.POS_KD, XCONF.ServoConf.POS_CMDILT,
            XCONF.ServoConf.GET_TEMP, XCONF.ServoConf.OVER_TEMP
        ]
        if servo_id is None or servo_id > self.axis:
            count = 7 if servo_id == 8 else self.axis
            pids = [[9999] * len(addrs) for _ in range(count)]
            for i in range(count):
                for j, addr in enumerate(addrs):
                    _, data = self.get_servo_addr_16(i + 1, addr)
                    if _ == 0:
                        pids[i][j] = data
        else:
            pids = [9999] * len(addrs)
            for j, addr in enumerate(addrs):
                _, data = self.get_servo_addr_16(servo_id, addr)
                pids[j] = data
        return 0, pids
