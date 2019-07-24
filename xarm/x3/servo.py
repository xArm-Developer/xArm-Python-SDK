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


class Servo(object):
    def __init__(self):
        pass

    @xarm_is_connected(_type='get')
    def get_servo_debug_msg(self, show=False, lang='en'):
        ret = self.arm_cmd.servo_get_dbmsg()
        dbmsg = []
        lang = lang if lang == 'cn' else 'en'
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
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
        logger.info('API -> set_servo_zero -> ret={}, servo_id={}'.format(ret[0], servo_id))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7, 'The value of parameter servo_id can only be 1-7.'
        assert addr is not None, 'The value of parameter addr cannot be None.'
        assert value is not None, 'The value of parameter value cannot be None.'
        ret = self.arm_cmd.servo_addr_w16(servo_id, addr, value)
        logger.info('API -> set_servo_addr_16 -> ret={}, servo_id={}, addr={}, value={}'.format(ret[0], servo_id, addr, value))
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_addr_16(self, servo_id=None, addr=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7, 'The value of parameter servo_id can only be 1-7.'
        assert addr is not None, 'The value of parameter addr cannot be None.'
        ret = self.arm_cmd.servo_addr_r16(servo_id, addr)
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_servo_addr_32(self, servo_id=None, addr=None, value=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7, 'The value of parameter servo_id can only be 1-7.'
        assert addr is not None, 'The value of parameter addr cannot be None.'
        assert value is not None, 'The value of parameter value cannot be None.'
        ret = self.arm_cmd.servo_addr_w32(servo_id, addr, value)
        logger.info('API -> set_servo_addr_32 -> ret={}, servo_id={}, addr={}, value={}'.format(ret[0], servo_id, addr, value))
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_addr_32(self, servo_id=None, addr=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
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
