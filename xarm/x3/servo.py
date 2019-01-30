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
from ..core.utils.log import pretty_print


class Servo(object):
    def __init__(self):
        pass

    @xarm_is_connected(_type='get')
    def get_servo_debug_msg(self, show=False):
        ret = self.arm_cmd.servo_get_dbmsg()
        dbmsg = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            for i in range(1, 9):
                servo_error = ServoError(ret[i * 2])
                dbmsg.append({
                    'name': '伺服(Servo)-{}'.format(i) if i < 8 else '机械爪(Gripper)',
                    'servo_id': i,
                    'status': ret[i * 2 - 1],
                    'error': {
                        'code': ret[i * 2],
                        'desc': servo_error.description if ret[i * 2 - 1] != 3 else {'cn': '通信错误', 'en': 'Communication error'},
                        'handle': servo_error.handle if ret[i * 2 - 1] != 3 else ['检查连接，重新上电']
                    }
                })
        if show:
            pretty_print('************GetServoDebugMsg, Status: {}*************'.format(ret[0]), color='light_blue')
            for servo_info in dbmsg:
                color = 'red' if servo_info['error']['code'] != 0 or servo_info['status'] != 0 else 'white'
                pretty_print('* {}, Status: {}, Code: {}'.format(
                    servo_info['name'], servo_info['status'],
                    servo_info['error']['code']), color=color)
                if servo_info['error']['desc']:
                    pretty_print('*  Description: {}({})'.format(servo_info['error']['desc']['cn'], servo_info['error']['desc']['en']), color=color)
                if servo_info['error']['handle']:
                    pretty_print('*  Handle: {}'.format(servo_info['error']['handle']), color=color)
            pretty_print('*' * 50, color='light_blue')
        return ret[0], dbmsg

        # if show:
        #     if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
        #         print('=' * 50)
        #         for i in range(1, 8):
        #             if ret[i * 2 - 1] != 0:
        #                 servo_error = ServoError(ret[i * 2])
        #                 if ret[i * 2 - 1] == 3:
        #                     servo_error.description = '通信错误'
        #                 err_code = '{} ({})'.format(hex(ret[i * 2]), ret[i * 2])
        #                 print('伺服{}, 状态: {}, 错误码: {}, 错误信息: {}'.format(
        #                     i, ret[i * 2 - 1], err_code, servo_error.description))
        #                 print('处理方法: {}'.format(servo_error.handle))
        #             else:
        #                 print('伺服{}, 状态: {}, 错误码: 0'.format(i, ret[i * 2 - 1]))
        #         if ret[15] != 0:
        #             servo_error = ServoError(ret[16])
        #             if ret[15] == 3:
        #                 servo_error.description = '通信错误'
        #             err_code = '{} ({})'.format(hex(ret[16]), ret[16])
        #             print('机械爪, 状态: {}, 错误码: {}, 错误信息: {}'.format(
        #                 ret[15], err_code, servo_error.description))
        #             print('处理方法: {}'.format(servo_error.handle))
        #         else:
        #             print('机械爪, 状态: {}, 错误码: 0'.format(ret[15]))
        #         print('=' * 50)
        # return ret

    @xarm_is_connected(_type='set')
    def set_servo_zero(self, servo_id=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8, 'The value of parameter servo_id can only be 1-8.'
        ret = self.arm_cmd.servo_set_zero(servo_id)
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

