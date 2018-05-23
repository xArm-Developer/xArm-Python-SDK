#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from .utils import xarm_is_connected


class Gripper(object):
    def __init__(self):
        pass

    @xarm_is_connected
    def gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_en(int(enable))
        return ret[0]

    @xarm_is_connected
    def get_gripper_position(self):
        ret = self.arm_cmd.gripper_get_pos()
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
