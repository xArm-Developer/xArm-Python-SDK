#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import re

GCODE_PARAM_X = 'X'  # TCP-X
GCODE_PARAM_Y = 'Y'  # TCP-Y
GCODE_PARAM_Z = 'Z'  # TCP-Z
GCODE_PARAM_A = 'A'  # TCP-Roll
GCODE_PARAM_B = 'B'  # TCP-Pitch
GCODE_PARAM_C = 'C'  # TCP-Yaw
GCODE_PARAM_R = 'R'  # TCP-Radius
GCODE_PARAM_I = 'I'  # Joint-1
GCODE_PARAM_J = 'J'  # Joint-2
GCODE_PARAM_K = 'K'  # Joint-3
GCODE_PARAM_L = 'L'  # Joint-4
GCODE_PARAM_M = 'M'  # Joint-5
GCODE_PARAM_N = 'N'  # Joint-6
GCODE_PARAM_O = 'O'  # Joint-7
GCODE_PARAM_F = 'F'  # Move-Speed
GCODE_PARAM_Q = 'Q'  # Move-Acc
GCODE_PARAM_T = 'T'  # Move-Time
GCODE_PARAM_V = 'V'  # Value
GCODE_PARAM_D = 'D'  # Addr


class GcodeParser:
    def __init__(self):
        self._int_val = 0
        self._float_val = 0.0

    @staticmethod
    def __get_value(string, ch, return_type, default=None):
        pattern = r'{}(\-?\d+\.?\d*)'.format(ch)
        data = re.findall(pattern, string)
        if len(data) > 0:
            return return_type(data[0])
        return default

    @staticmethod
    def __get_hex_value(string, ch, default=None):
        pattern = r'{}(-?\w{{3,4}})'.format(ch)
        data = re.findall(pattern, string)
        if len(data) > 0:
            return int(data[0], base=16)
        return default

    def _get_int_value(self, string, ch, default=None):
        return self.__get_value(string, ch, int, default=default)

    def _get_float_value(self, string, ch, default=None):
        return self.__get_value(string, ch, float, default=default)

    def get_int_value(self, string, default=None):
        if default is None:
            default = self._int_val
            self._int_val = self._get_int_value(string, GCODE_PARAM_V, default=default)
            return self._int_val
        else:
            return self._get_int_value(string, GCODE_PARAM_V, default=default)

    def get_float_value(self, string, default=0):
        return self._get_float_value(string, GCODE_PARAM_V, default=default)

    def get_addr(self, string, default=0):
        return self.__get_hex_value(string, GCODE_PARAM_D, default=default)

    def get_gcode_cmd_num(self, string, ch):
        return self._get_int_value(string, ch, default=-1)

    def get_mvvelo(self, string, default=None):
        return self._get_float_value(string, GCODE_PARAM_F, default=default)

    def get_mvacc(self, string, default=None):
        return self._get_float_value(string, GCODE_PARAM_Q, default=default)

    def get_mvtime(self, string, default=None):
        return self._get_float_value(string, GCODE_PARAM_T, default=default)

    def get_mvradius(self, string, default=None):
        return self._get_float_value(string, GCODE_PARAM_R, default=default)

    def get_id_num(self, string, default=None):
        return self._get_int_value(string, GCODE_PARAM_I, default=default)

    def get_poses(self, string, default=None):
        pose = [None] * 6
        pose[0] = self._get_float_value(string[2:], GCODE_PARAM_X, default=default)
        pose[1] = self._get_float_value(string[2:], GCODE_PARAM_Y, default=default)
        pose[2] = self._get_float_value(string[2:], GCODE_PARAM_Z, default=default)
        pose[3] = self._get_float_value(string[2:], GCODE_PARAM_A, default=default)
        pose[4] = self._get_float_value(string[2:], GCODE_PARAM_B, default=default)
        pose[5] = self._get_float_value(string[2:], GCODE_PARAM_C, default=default)
        return pose

    def get_joints(self, string, default=None):
        joints = [None] * 7
        joints[0] = self._get_float_value(string[2:], GCODE_PARAM_I, default=default)
        joints[1] = self._get_float_value(string[2:], GCODE_PARAM_J, default=default)
        joints[2] = self._get_float_value(string[2:], GCODE_PARAM_K, default=default)
        joints[3] = self._get_float_value(string[2:], GCODE_PARAM_L, default=default)
        joints[4] = self._get_float_value(string[2:], GCODE_PARAM_M, default=default)
        joints[5] = self._get_float_value(string[2:], GCODE_PARAM_N, default=default)
        joints[6] = self._get_float_value(string[2:], GCODE_PARAM_O, default=default)
        return joints
