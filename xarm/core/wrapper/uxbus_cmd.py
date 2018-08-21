#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import threading
import functools
from ..utils import convert
from ..config.x_config import XCONF


def lock_require(func):
    @functools.wraps(func)
    def decorator(*args, **kwargs):
        with args[0].lock:
            return func(*args, **kwargs)
    return decorator


class UxbusCmd(object):
    def __init__(self):
        self._has_error = False
        self._has_warn = False
        self._error_code = 0
        self._warn_code = 0
        self._cmd_num = 0
        self.lock = threading.Lock()

    def check_xbus_prot(self, data, funcode):
        raise NotImplementedError

    def send_pend(self, funcode, num, timeout):
        raise NotImplementedError

    def send_xbus(self, funcode, txdata, num):
        raise NotImplementedError

    @lock_require
    def set_nu8(self, funcode, datas, num):
        ret = self.send_xbus(funcode, datas, num)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.send_pend(funcode, 0, XCONF.UxbusConf.SET_TIMEOUT)

    @lock_require
    def get_nu8(self, funcode, num):
        ret = self.send_xbus(funcode, 0, 0)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num + 1)
        return self.send_pend(funcode, num, XCONF.UxbusConf.GET_TIMEOUT)

    @lock_require
    def set_nfp32(self, funcode, datas, num):
        hexdata = convert.fp32s_to_bytes(datas, num)
        ret = self.send_xbus(funcode, hexdata, num * 4)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.send_pend(funcode, 0, XCONF.UxbusConf.SET_TIMEOUT)

    @lock_require
    def get_nfp32(self, funcode, num):
        ret = self.send_xbus(funcode, 0, 0)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num * 4 + 1)
        ret = self.send_pend(funcode, num * 4, XCONF.UxbusConf.GET_TIMEOUT)
        data = [0] * (1 + num)
        data[0] = ret[0]
        data[1:num] = convert.bytes_to_fp32s(ret[1:num * 4 + 1], num)
        return data

    @lock_require
    def swop_nfp32(self, funcode, datas, txn, rxn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_xbus(funcode, hexdata, txn * 4)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (rxn + 1)
        ret = self.send_pend(funcode, rxn * 4, XCONF.UxbusConf.GET_TIMEOUT)
        data = [0] * (1 + rxn)
        data[0] = ret[0]
        data[1:rxn] = convert.bytes_to_fp32s(ret[1:rxn * 4 + 1], rxn)
        return data

    @lock_require
    def is_nfp32(self, funcode, datas, txn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_xbus(funcode, hexdata, txn * 4)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * 2
        return self.send_pend(funcode, 1, XCONF.UxbusConf.GET_TIMEOUT)

    @lock_require
    def get_nu16(self, funcode, num):
        ret = self.send_xbus(funcode, 0, 0)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num * 2 + 1)
        ret = self.send_pend(funcode, num * 2, XCONF.UxbusConf.GET_TIMEOUT)
        data = [0] * (1 + num)
        data[0] = ret[0]
        data[1:num] = convert.bytes_to_u16s(ret[1:num * 2 + 1], num)
        return data

    def get_version(self):
        return self.get_nu8(XCONF.UxbusReg.GET_VERSION, 40)

    def motion_en(self, axis_id, enable):
        txdata = [axis_id, int(enable)]
        return self.set_nu8(XCONF.UxbusReg.MOTION_EN, txdata, 2)

    def set_state(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_STATE, txdata, 1)

    def get_state(self):
        return self.get_nu8(XCONF.UxbusReg.GET_STATE, 1)

    def get_cmdnum(self):
        return self.get_nu16(XCONF.UxbusReg.GET_CMDNUM, 1)

    def get_err_code(self):
        return self.get_nu8(XCONF.UxbusReg.GET_ERROR, 2)

    def clean_err(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_ERR, 0, 0)

    def clean_war(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_WAR, 0, 0)

    def set_brake(self, axis_id, enable):
        txdata = [axis_id, int(enable)]
        return self.set_nu8(XCONF.UxbusReg.SET_BRAKE, txdata, 2)

    def move_line(self, mvpose, mvvelo, mvacc, mvtime):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_LINE, txdata, 9)

    def move_lineb(self, mvpose, mvvelo, mvacc, mvtime, mvradii):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime, mvradii]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_LINEB, txdata, 10)

    def move_joint(self, mvjoint, mvvelo, mvacc, mvtime):
        txdata = [mvjoint[i] for i in range(7)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_JOINT, txdata, 10)

    def move_gohome(self, mvvelo, mvacc, mvtime):
        txdata = [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_HOME, txdata, 3)

    def sleep_instruction(self, sltime):
        txdata = [sltime]
        return self.set_nfp32(XCONF.UxbusReg.SLEEP_INSTT, txdata, 1)

    def set_tcp_jerk(self, jerk):
        txdata = [jerk]
        return self.set_nfp32(XCONF.UxbusReg.SET_TCP_JERK, txdata, 1)

    def set_tcp_maxacc(self, acc):
        txdata = [acc]
        return self.set_nfp32(XCONF.UxbusReg.SET_TCP_MAXACC, txdata, 1)

    def set_joint_jerk(self, jerk):
        txdata = [jerk]
        return self.set_nfp32(XCONF.UxbusReg.SET_JOINT_JERK, txdata, 1)

    def set_joint_maxacc(self, acc):
        txdata = [acc]
        return self.set_nfp32(XCONF.UxbusReg.SET_JOINT_MAXACC, txdata, 1)

    def set_tcp_offset(self, pose_offset):
        return self.set_nfp32(XCONF.UxbusReg.SET_TCP_OFFSET, pose_offset, 6)

    def clean_conf(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_CONF, 0, 0)

    def save_conf(self):
        return self.set_nu8(XCONF.UxbusReg.SAVE_CONF, 0, 0)

    def get_joint_pos(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_JOINT_POS, 7)

    def get_tcp_pose(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_TCP_POSE, 6)

    def get_ik(self, pose):
        return self.swop_nfp32(XCONF.UxbusReg.GET_IK, pose, 6, 7)

    def get_fk(self, angles):
        return self.swop_nfp32(XCONF.UxbusReg.GET_FK, angles, 7, 6)

    def is_joint_limit(self, joint):
        return self.is_nfp32(XCONF.UxbusReg.IS_JOINT_LIMIT, joint, 7)

    def is_tcp_limit(self, pose):
        return self.is_nfp32(XCONF.UxbusReg.IS_TCP_LIMIT, pose, 6)

    def gripper_addr_w16(self, addr, value):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.GRIPP_W16B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.GRIPP_W16B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def gripper_addr_r16(self, addr):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.GRIPP_R16B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.GRIPP_R16B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]

    def gripper_addr_w32(self, addr, value):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.GRIPP_W32B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.GRIPP_W32B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def gripper_addr_r32(self, addr):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.GRIPP_R32B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.GRIPP_R32B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]

    def gripper_set_en(self, value):
        return self.gripper_addr_w16(XCONF.ServoConf.CON_EN, value)

    def gripper_set_mode(self, value):
        return self.gripper_addr_w16(XCONF.ServoConf.CON_MODE, value)

    def gripper_set_zero(self):
        return self.gripper_addr_w16(XCONF.ServoConf.MT_ZERO, 1)

    def gripper_get_pos(self):
        return self.gripper_addr_r32(XCONF.ServoConf.CURR_POS)

    def gripper_set_pos(self, pulse):
        return self.gripper_addr_w32(XCONF.ServoConf.TAGET_POS, pulse)

    def gripper_set_posspd(self, speed):
        return self.gripper_addr_w16(XCONF.ServoConf.POS_SPD, speed)

    def gripper_get_errcode(self):
        ret = self.get_nu8(XCONF.UxbusReg.GPGET_ERR, 2)
        return ret

    def gripper_clean_err(self):
        return self.gripper_addr_w16(XCONF.ServoConf.RESET_ERR, 1)

    def servo_set_zero(self, axis_id):
        txdata = [int(axis_id)]
        ret = self.set_nu8(XCONF.UxbusReg.SERVO_ZERO, txdata, 1)
        return ret

    def servo_get_dbmsg(self):
        ret = self.get_nu8(XCONF.UxbusReg.SERVO_DBMSG, 16)
        return ret

    def servo_addr_w16(self, axis_id, addr, value):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_W16B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_W16B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def servo_addr_r16(self, axis_id, addr):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_R16B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_R16B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]
        # return [ret[0], convert.bytes_to_long_big(ret[1:5])[0]]

    def servo_addr_w32(self, axis_id, addr, value):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_W32B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_W32B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def servo_addr_r32(self, axis, addr):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_R32B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_R32B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]
        # return [ret[0], convert.bytes_to_long_big(ret[1:5])[0]]

