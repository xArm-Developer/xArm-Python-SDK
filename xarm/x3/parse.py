#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import re

GCODE_PARAM_X = 'X'
GCODE_PARAM_Y = 'Y'
GCODE_PARAM_Z = 'Z'
GCODE_PARAM_A = 'A'
GCODE_PARAM_B = 'B'
GCODE_PARAM_C = 'C'
GCODE_PARAM_R = 'R'
GCODE_PARAM_I = 'I'
GCODE_PARAM_J = 'J'
GCODE_PARAM_K = 'K'
GCODE_PARAM_L = 'L'
GCODE_PARAM_M = 'M'
GCODE_PARAM_N = 'N'
GCODE_PARAM_O = 'O'
GCODE_PARAM_F = 'F'
GCODE_PARAM_Q = 'Q'
GCODE_PARAM_T = 'T'
GCODE_PARAM_E = 'E'
GCODE_PARAM_V = 'V'
GCODE_PARAM_W = 'W'
GCODE_PARAM_S = 'S'

RAD_DEGREE = 57.295779513082320876798154814105


def gcode_get_chfp(str, ch):
    relink2 = ch + '\d+\.?\d*'
    data = re.findall(relink2, str)
    if len(data) > 0:
        return (float(data[0].split(ch)[1]))
    else:
        return 0.0


def gcode_get_chint(str, ch):
    relink2 = ch + '\-?\d+\.?\d*'
    data = re.findall(relink2, str)
    if len(data) > 0:
        return (int(data[0].split(ch)[1]))
    else:
        return -1


def __gcode_get_numfp(str, param):
    relink2 = param + '\-?\d+\.?\d*' if param != GCODE_PARAM_Q else '[QE]\-?\d+\.?\d*'
    data = re.findall(relink2, str)
    if len(data) > 0:
        if 'E' in data[0]:
            value = (float(data[0].split('E')[1]))
        else:
            value = (float(data[0].split(param)[1]))
        return value
    else:
        return None


def gcode_get_mvvelo(str):
    return __gcode_get_numfp(str, GCODE_PARAM_F)


def gcode_get_mvacc(str):
    return __gcode_get_numfp(str, GCODE_PARAM_Q)


def gcode_get_mvtime(str):
    return __gcode_get_numfp(str, GCODE_PARAM_T)


def gcode_get_mvradii(str):
    return __gcode_get_numfp(str, GCODE_PARAM_R)


def gcode_get_value(str):
    return __gcode_get_numfp(str, GCODE_PARAM_V)


def gcode_get_servo(str):
    servo_id = gcode_get_chint(str, GCODE_PARAM_S)
    if servo_id < 1:
        servo_id = None
    return servo_id


def gcode_get_wait(str):
    return gcode_get_chint(str, GCODE_PARAM_W) > 0


def gcode_get_mvcarts(str):
    pose = [0, 0, 0, 0, 0, 0]
    pose[0] = __gcode_get_numfp(str[2:], GCODE_PARAM_X)
    pose[1] = __gcode_get_numfp(str[2:], GCODE_PARAM_Y)
    pose[2] = __gcode_get_numfp(str[2:], GCODE_PARAM_Z)
    pose[3] = __gcode_get_numfp(str[2:], GCODE_PARAM_A)  # / RAD_DEGREE
    pose[4] = __gcode_get_numfp(str[2:], GCODE_PARAM_B)  # / RAD_DEGREE
    pose[5] = __gcode_get_numfp(str[2:], GCODE_PARAM_C)  # / RAD_DEGREE
    return pose


def gcode_get_mvjoints(str):
    joint = [0, 0, 0, 0, 0, 0, 0]
    joint[0] = __gcode_get_numfp(str[2:], GCODE_PARAM_I)  # / RAD_DEGREE
    joint[1] = __gcode_get_numfp(str[2:], GCODE_PARAM_J)  # / RAD_DEGREE
    joint[2] = __gcode_get_numfp(str[2:], GCODE_PARAM_K)  # / RAD_DEGREE
    joint[3] = __gcode_get_numfp(str[2:], GCODE_PARAM_L)  # / RAD_DEGREE
    joint[4] = __gcode_get_numfp(str[2:], GCODE_PARAM_M)  # / RAD_DEGREE
    joint[5] = __gcode_get_numfp(str[2:], GCODE_PARAM_N)  # / RAD_DEGREE
    joint[6] = __gcode_get_numfp(str[2:], GCODE_PARAM_O)  # / RAD_DEGREE
    return joint
