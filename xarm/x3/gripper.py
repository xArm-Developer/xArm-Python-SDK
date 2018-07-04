#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from .utils import xarm_is_connected
from ..core.config import x2_config


class Gripper(object):
    def __init__(self):
        self.gripper_pos = 0
        pass

    @xarm_is_connected
    def gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_set_en(int(enable))
        return ret[0]

    @xarm_is_connected
    def set_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_set_mode(mode)
        return ret

    @xarm_is_connected
    def get_gripper_position(self):
        ret = self.arm_cmd.gripper_get_pos()
        if ret[0] in [0, x2_config.UX2_ERR_CODE, x2_config.UX2_WAR_CODE] and len(ret) > 1:
            ret[1] = float('{:.2f}'.format(ret[1][0]))
        return ret

    @xarm_is_connected
    def set_gripper_position(self, pos):
        ret = self.arm_cmd.gripper_set_pos(pos)
        return ret

    @xarm_is_connected
    def set_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_set_posspd(speed)
        return ret

    @xarm_is_connected
    def get_gripper_err_code(self):
        ret = self.arm_cmd.gripper_get_errcode()
        return ret

    @xarm_is_connected
    def clean_gripper_error(self):
        ret = self.arm_cmd.gripper_clean_err()
        return ret

    @xarm_is_connected
    def set_gripper_zero(self):
        ret = self.arm_cmd.gripper_set_zero()
        return ret
