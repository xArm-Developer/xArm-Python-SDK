#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./instruction_function/x2_hexcmd.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#

import threading
import functools
from ..utils import convert
from ..config import x2_config
from ..config import x2_cmd_config


def lock_require(func):
    @functools.wraps(func)
    def decorator(*args, **kwargs):
        with args[0].lock:
            return func(*args, **kwargs)
    return decorator


class X2HexCmd(object):
    def __init__(self):
        self.DB_FLG = '[x2 hcmd] '
        self._has_error = False
        self._has_warn = False
        self._error_code = 0
        self._warn_code = 0
        self._cmd_num = 0
        self.lock = threading.Lock()

    def check_xbus_proc(self, data, funcode):
        raise NotImplementedError

    def send_pend(self, funcode, n, timeout):
        raise NotImplementedError

    def send_xbus(self, funcode, txdata, num):
        raise NotImplementedError

    @lock_require
    def set_nu8(self, funcode, datas, n):
        ret = self.send_xbus(funcode, datas, n)
        if 0 != ret:
            return [x2_config.UX2_ERR_NOTTCP]
        return self.send_pend(funcode, 0, x2_config.UX2_SET_TIMEOUT)

    @lock_require
    def get_nu8(self, funcode, n):
        ret = self.send_xbus(funcode, 0, 0)
        if 0 != ret:
            return [x2_config.UX2_ERR_NOTTCP] * (n + 1)
        return self.send_pend(funcode, n, x2_config.UX2_GET_TIMEOUT)

    @lock_require
    def set_nfp32(self, funcode, datas, n):
        hexdata = convert.fp32s_to_bytes(datas, n)
        ret = self.send_xbus(funcode, hexdata, n * 4)
        if 0 != ret:
            return [x2_config.UX2_ERR_NOTTCP]
        return self.send_pend(funcode, 0, x2_config.UX2_SET_TIMEOUT)

    @lock_require
    def get_nfp32(self, funcode, n):
        ret = self.send_xbus(funcode, 0, 0)
        if 0 != ret:
            return [x2_config.UX2_ERR_NOTTCP] * (n * 4 + 1)
        ret = self.send_pend(funcode, n * 4, x2_config.UX2_GET_TIMEOUT)
        data = [0] * (1 + n)
        data[0] = ret[0]
        data[1:n] = convert.bytes_to_fp32s(ret[1:n * 4 + 1], n)
        return data

    @lock_require
    def swop_nfp32(self, funcode, datas, txn, rxn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_xbus(funcode, hexdata, txn * 4)
        if 0 != ret:
            return [x2_config.UX2_ERR_NOTTCP] * (rxn + 1)
        ret = self.send_pend(funcode, rxn * 4, x2_config.UX2_GET_TIMEOUT)
        data = [0] * (1 + rxn)
        data[0] = ret[0]
        data[1:rxn] = convert.bytes_to_fp32s(ret[1:rxn * 4 + 1], rxn)
        return data

    @lock_require
    def is_nfp32(self, funcode, datas, txn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_xbus(funcode, hexdata, txn * 4)
        if 0 != ret:
            return [x2_config.UX2_ERR_NOTTCP] * 2
        return self.send_pend(funcode, 1, x2_config.UX2_GET_TIMEOUT)

    @lock_require
    def get_nu16(self, funcode, n):
        ret = self.send_xbus(funcode, 0, 0)
        if 0 != ret:
            return [x2_config.UX2_ERR_NOTTCP] * (n * 2 + 1)
        ret = self.send_pend(funcode, n * 2, x2_config.UX2_GET_TIMEOUT)
        data = [0] * (1 + n)
        data[0] = ret[0]
        data[1:n] = convert.bytes_to_u16s(ret[1:n * 2 + 1], n)
        return data

    def get_version(self):
        return self.get_nu8(x2_cmd_config.BU2RG_GET_VERSION, 30)

    def motion_en(self, axis, enable):
        txdata = [axis, int(enable)]
        return self.set_nu8(x2_cmd_config.BU2RG_MOTION_EN, txdata, 2)

    def set_state(self, value):
        txdata = [value]
        return self.set_nu8(x2_cmd_config.BU2RG_SET_STATE, txdata, 1)

    def get_state(self):
        return self.get_nu8(x2_cmd_config.BU2RG_GET_STATE, 1)

    def get_cmdnum(self):
        return self.get_nu16(x2_cmd_config.BU2RG_GET_CMDNUM, 1)

    def get_err_code(self):
        return self.get_nu8(x2_cmd_config.BU2RG_GET_ERROR, 2)

    def clean_err(self):
        return self.set_nu8(x2_cmd_config.BU2RG_CLEAN_ERR, 0, 0)

    def clean_war(self):
        return self.set_nu8(x2_cmd_config.BU2RG_CLEAN_WAR, 0, 0)

    def set_brake(self, axis, enable):
        txdata = [axis, int(enable)]
        return self.set_nu8(x2_cmd_config.BU2RG_SET_BRAKE, txdata, 2)

    def move_line(self, mvpose, mvvelo, mvacc, mvtime):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(x2_cmd_config.BU2RG_MOVE_LINE, txdata, 9)

    def move_lineb(self, mvpose, mvvelo, mvacc, mvtime, mvradii):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime, mvradii]
        return self.set_nfp32(x2_cmd_config.BU2RG_MOVE_LINEB, txdata, 10)

    def move_joint(self, mvjoint, mvvelo, mvacc, mvtime):
        txdata = [mvjoint[i] for i in range(7)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(x2_cmd_config.BU2RG_MOVE_JOINT, txdata, 10)

    def move_gohome(self, mvvelo, mvacc, mvtime):
        txdata = [mvvelo, mvacc, mvtime]
        return self.set_nfp32(x2_cmd_config.BU2RG_MOVE_HOME, txdata, 3)

    def sleep_instruction(self, sltime):
        txdata = [sltime]
        return self.set_nfp32(x2_cmd_config.BU2RG_SLEEP_INSTT, txdata, 1)

    def set_tcp_jerk(self, jerk):
        txdata = [jerk]
        return self.set_nfp32(x2_cmd_config.BU2RG_SET_TCP_JERK, txdata, 1)

    def set_tcp_maxacc(self, acc):
        txdata = [acc]
        return self.set_nfp32(x2_cmd_config.BU2RG_SET_TCP_MAXACC, txdata, 1)

    def set_joint_jerk(self, jerk):
        txdata = [jerk]
        return self.set_nfp32(x2_cmd_config.BU2RG_SET_JOINT_JERK, txdata, 1)

    def set_joint_maxacc(self, acc):
        txdata = [acc]
        return self.set_nfp32(x2_cmd_config.BU2RG_SET_JOINT_MAXACC, txdata, 1)

    def set_tcp_offset(self, pose_offset):
        return self.set_nfp32(x2_cmd_config.BU2RG_SET_TCP_OFFSET, pose_offset, 6)

    def clean_conf(self):
        return self.set_nu8(x2_cmd_config.BU2RG_CLEAN_CONF, 0, 0)

    def save_conf(self):
        return self.set_nu8(x2_cmd_config.BU2RG_SAVE_CONF, 0, 0)

    def get_joint_pos(self):
        return self.get_nfp32(x2_cmd_config.BU2RG_GET_JOINT_POS, 7)

    def get_tcp_pose(self):
        return self.get_nfp32(x2_cmd_config.BU2RG_GET_TCP_POSE, 6)

    def get_ik(self, pose):
        return self.swop_nfp32(x2_cmd_config.BU2RG_GET_IK, pose, 6, 7)

    def get_fk(self, angles):
        return self.swop_nfp32(x2_cmd_config.BU2RG_GET_FK, angles, 7, 6)

    def is_joint_limit(self, joint):
        return self.is_nfp32(x2_cmd_config.BU2RG_IS_JOINT_LIMIT, joint, 7)

    def is_tcp_limit(self, pose):
        return self.is_nfp32(x2_cmd_config.BU2RG_IS_TCP_LIMIT, pose, 6)

    def gripper_en(self, enable):
        txdata = [int(enable)]
        return self.set_nu8(x2_cmd_config.BU2RG_GPSET_MOTION, txdata, 1)

    def gripper_mode(self, value):
        txdata = [value]
        return self.set_nu8(x2_cmd_config.BU2RG_GPSET_MODE, txdata, 1)

    def gripper_get_pos(self):
        return self.get_nfp32(x2_cmd_config.BU2RG_GPGET_POS, 1)

    def gripper_set_pos(self, pulse):
        txdata = [pulse]
        return self.set_nfp32(x2_cmd_config.BU2RG_GPSET_POS, txdata, 1)

    def gripper_set_posspd(self, speed):
        txdata = [speed]
        return self.set_nfp32(x2_cmd_config.BU2RG_GPSET_POSSP, txdata, 1)

    def gripper_get_errcode(self):
        return self.get_nu8(x2_cmd_config.BU2RG_GPGET_ERR, 1)

    def gripper_clean_err(self):
        return self.set_nu8(x2_cmd_config.BU2RG_GPCLE_ERR, 0, 0)

    def servo_set_zero(self, axis):
        txdata = [int(axis)]
        ret = self.set_nu8(x2_cmd_config.BU2RG_SERVO_ZERO, txdata, 1)
        return ret

    def servo_get_dbmsg(self):
        ret = self.get_nu8(x2_cmd_config.BU2RG_SERVO_DBMSG, 16)
        return ret

    def servo_addr_w16(self, axis, addr, value):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(x2_cmd_config.BU2RG_SERVO_W16B, txdata, 7)
        if ret != 0:
            return [x2_config.UX2_ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(x2_cmd_config.BU2RG_SERVO_W16B, 0, x2_config.UX2_GET_TIMEOUT)
        return ret

    def servo_addr_r16(self, axis, addr):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(x2_cmd_config.BU2RG_SERVO_R16B, txdata, 3)
        if ret != 0:
            return [x2_config.UX2_ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(x2_cmd_config.BU2RG_SERVO_R16B, 4, x2_config.UX2_GET_TIMEOUT)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        ret1[1] = convert.bytes_to_long_big(ret[1:5])[0]
        return ret1

    def servo_addr_w32(self, axis, addr, value):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(x2_cmd_config.BU2RG_SERVO_W32B, txdata, 7)
        if ret != 0:
            return [x2_config.UX2_ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(x2_cmd_config.BU2RG_SERVO_W32B, 0, x2_config.UX2_GET_TIMEOUT)
        return ret

    def servo_addr_r32(self, axis, addr):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(x2_cmd_config.BU2RG_SERVO_R32B, txdata, 3)
        if ret != 0:
            return [x2_config.UX2_ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(x2_cmd_config.BU2RG_SERVO_R32B, 4, x2_config.UX2_GET_TIMEOUT)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        ret1[1] = convert.bytes_to_long_big(ret[1:5])[0]
        return ret1
