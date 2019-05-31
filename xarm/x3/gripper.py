#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from .utils import xarm_is_connected
from ..core.config.x_config import XCONF


class Gripper(object):
    def __init__(self):
        self._gripper_error_code = 0

    # @xarm_is_connected(_type='set')
    # def set_gripper_addr_16(self, addr, value):
    #     ret = self.arm_cmd.gripper_addr_w16(addr, value)
    #     return ret[0]
    #
    # @xarm_is_connected(_type='get')
    # def get_gripper_addr_16(self, addr):
    #     ret = self.arm_cmd.gripper_addr_r16(addr)
    #     return ret[0], ret[1]
    #
    # @xarm_is_connected(_type='set')
    # def set_gripper_addr_32(self, addr, value):
    #     ret = self.arm_cmd.gripper_addr_w32(addr, value)
    #     return ret[0]
    #
    # @xarm_is_connected(_type='get')
    # def get_gripper_addr_32(self, addr):
    #     ret = self.arm_cmd.gripper_addr_r32(addr)
    #     return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_modbus_set_en(int(enable))
        _, err = self.get_gripper_err_code()
        return err if _ == 0 and err != 0 else ret[0]

    @xarm_is_connected(_type='set')
    def set_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_modbus_set_mode(mode)
        _, err = self.get_gripper_err_code()
        return err if _ == 0 and err != 0 else ret[0]

    @xarm_is_connected(_type='get')
    def get_gripper_position(self):
        ret = self.arm_cmd.gripper_modbus_get_pos()
        _, err = self.get_gripper_err_code()
        err = -1 if err is None else err
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 1:
            ret[0] = err if _ == 0 and err != 0 else ret[0]
            ret[1] = float('{:.2f}'.format(ret[1]))
            # if ret[0] in [XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] or err != 0:
            #     # self.get_err_warn_code()
            #     if self.error_code != 19:
            #         ret[0] = 0
            #         ret[1] = float('{:.2f}'.format(ret[1]))
            #     else:
            #         ret[0] = XCONF.UxbusState.ERR_CODE
            #         ret[1] = float('{:.2f}'.format(ret[1]))
            # else:
            #     ret[0] = 0
            #     ret[1] = float('{:.2f}'.format(ret[1]))
            return ret[0], ret[1]
        return ret[0], None

    @xarm_is_connected(_type='set')
    def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):
        if auto_enable:
            self.arm_cmd.gripper_modbus_set_en(True)
        if speed is not None:
            self.arm_cmd.gripper_modbus_set_posspd(speed)
        is_add = True
        last_pos = 0
        code, ret = self.get_gripper_position()
        if code in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and ret is not None:
            last_pos = int(ret)
            if last_pos == pos:
                return 0
            is_add = True if pos > last_pos else False
        code = self.arm_cmd.gripper_modbus_set_pos(pos)
        if wait:
            count = 0
            start_time = time.time()
            if not timeout or not isinstance(timeout, (int, float)):
                timeout = 10
            while time.time() - start_time < timeout:
                ret = self.get_gripper_position()
                if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and ret[1] is not None:
                    cur_pos = int(ret[1])
                    if abs(pos - cur_pos) < 1:
                        last_pos = cur_pos
                        break
                    if is_add:
                        if cur_pos <= last_pos:
                            count += 1
                        elif cur_pos <= pos:
                            last_pos = cur_pos
                            count = 0
                    else:
                        if cur_pos >= last_pos:
                            count += 1
                        elif cur_pos >= pos:
                            last_pos = cur_pos
                            count = 0
                    if count >= 15:
                        print('gripper target: {}, current: {}'.format(pos, cur_pos))
                        break
                    time.sleep(0.1)
                else:
                    code = ret
                    break
            print('gripper, pos: {}, last: {}'.format(pos, last_pos))
            return code[0]
        else:
            _, err = self.get_gripper_err_code()
            return err if _ == 0 and err != 0 else code[0]

    @xarm_is_connected(_type='set')
    def set_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_modbus_set_posspd(speed)
        _, err = self.get_gripper_err_code()
        return err if _ == 0 and err != 0 else ret[0]

    @xarm_is_connected(_type='get')
    def get_gripper_err_code(self):
        ret = self.arm_cmd.gripper_modbus_get_errcode()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
            self._gripper_error_code = ret[1]
            return ret[0], ret[1]
        return ret[0], 0

    @xarm_is_connected(_type='set')
    def clean_gripper_error(self):
        ret = self.arm_cmd.gripper_modbus_clean_err()
        _, err = self.get_gripper_err_code()
        return err if _ == 0 and err != 0 else ret[0]

    @xarm_is_connected(_type='set')
    def set_gripper_zero(self):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        ret = self.arm_cmd.gripper_modbus_set_zero()
        _, err = self.get_gripper_err_code()
        return err if _ == 0 and err != 0 else ret[0]
