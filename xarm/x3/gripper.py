#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from .utils import xarm_is_connected, xarm_is_ready
from ..core.config.x_config import XCONF


class Gripper(object):
    def __init__(self):
        self.gripper_pos = 0
        pass

    @xarm_is_connected(_type='set')
    def set_gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_set_en(int(enable))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_set_mode(mode)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_gripper_position(self):
        ret = self.arm_cmd.gripper_get_pos()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 1:
            if ret[0] in [XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
                self.get_err_warn_code()
                if self.error_code != 28:
                    ret[0] = 0
                    ret[1] = float('{:.2f}'.format(ret[1]))
                else:
                    ret[0] = XCONF.UxbusState.ERR_CODE
                    ret[1] = None
            else:
                ret[0] = 0
                ret[1] = float('{:.2f}'.format(ret[1]))
            return ret[0], ret[1]
        return ret[0], None

    @xarm_is_connected(_type='set')
    def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):
        if auto_enable:
            self.arm_cmd.gripper_set_en(True)
        if speed is not None:
            self.arm_cmd.gripper_set_posspd(speed)
        is_add = True
        last_pos = 0
        ret = self.arm_cmd.gripper_get_pos()
        if ret[0] != 0:
            self.get_err_warn_code()
        if self.error_code != 28:
            last_pos = int(ret[1])
            if last_pos == pos:
                return 0
            is_add = True if pos > last_pos else False
        code = self.arm_cmd.gripper_set_pos(pos)
        if wait:
            count = 0
            start_time = time.time()
            if not timeout or not isinstance(timeout, (int, float)):
                timeout = 10
            while self.error_code != 28 and time.time() - start_time < timeout:
                ret = self.arm_cmd.gripper_get_pos()
                if ret[0] != 0:
                    self.get_err_warn_code()
                if self.error_code != 28:
                    cur_pos = int(ret[1])
                    # print(cur_pos, last_pos)
                    if abs(pos - cur_pos) < 1:
                        break
                    if is_add:
                        if cur_pos <= last_pos:
                            count += 1
                        else:
                            last_pos = cur_pos
                            count = 0
                    else:
                        if cur_pos >= last_pos:
                            count += 1
                        else:
                            last_pos = cur_pos
                            count = 0
                    if count >= 25:
                        print('gripper target: {}, current: {}'.format(pos, cur_pos))
                        break
                time.sleep(0.02)
            print('gripper, pos: {}, last: {}'.format(pos, last_pos))
        return code[0]

    @xarm_is_connected(_type='set')
    def set_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_set_posspd(speed)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_gripper_err_code(self):
        ret = self.arm_cmd.gripper_get_errcode()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
            return ret[0], ret[1]
        return ret[0], None

    @xarm_is_connected(_type='set')
    def clean_gripper_error(self):
        ret = self.arm_cmd.gripper_clean_err()
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_gripper_zero(self):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        ret = self.arm_cmd.gripper_set_zero()
        return ret[0]
