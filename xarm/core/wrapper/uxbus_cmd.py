#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
import math
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
    BAUDRATES = (4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
                 1000000, 1500000, 2000000, 2500000)

    def __init__(self, set_feedback_key_tranid=None):
        self._has_error = False
        self._has_warn = False
        self._state_is_ready = False
        self._error_code = 0
        self._warn_code = 0
        self._cmd_num = 0
        self._debug = False
        self.lock = threading.Lock()
        self._G_TOUT = XCONF.UxbusConf.GET_TIMEOUT / 1000
        self._S_TOUT = XCONF.UxbusConf.SET_TIMEOUT / 1000
        self._last_comm_time = time.monotonic()
        self._last_modbus_comm_time = time.monotonic()
        self._feedback_type = 0
        self._set_feedback_key_tranid = set_feedback_key_tranid

    @property
    def last_comm_time(self):
        return self._last_comm_time

    @property
    def state_is_ready(self):
        return self._state_is_ready
    
    def _get_trans_id(self):
        return 0

    def set_timeout(self, timeout):
        try:
            if isinstance(timeout, (tuple, list)):
                if len(timeout) >= 2:
                    self._S_TOUT = timeout[0] if timeout[0] > 0 else self._S_TOUT
                    self._G_TOUT = timeout[1] if timeout[1] > 0 else self._G_TOUT
                elif len(timeout) == 1:
                    self._S_TOUT = timeout[0] if timeout[0] > 0 else self._S_TOUT
                    self._G_TOUT = timeout[0] if timeout[0] > 0 else self._G_TOUT
            elif isinstance(timeout, (int, float)):
                self._S_TOUT = timeout if timeout > 0 else self._S_TOUT
                self._G_TOUT = timeout if timeout > 0 else self._G_TOUT
        except:
            pass
        return [self._S_TOUT, self._G_TOUT] if self._S_TOUT != self._G_TOUT else self._S_TOUT

    def set_debug(self, debug):
        self._debug = debug
    
    def send_modbus_request(self, unit_id, pdu_data, pdu_len, prot_id=-1, t_id=None):
        raise NotImplementedError
    
    def recv_modbus_response(self, t_unit_id, t_trans_id, num, timeout, t_prot_id=-1, ret_raw=False):
        raise NotImplementedError

    @lock_require
    def set_nu8(self, funcode, datas, num, timeout=None, feedback_key=None, feedback_type=XCONF.FeedbackType.MOTION_FINISH):
        need_set_fb = feedback_type != 0 and (self._feedback_type & feedback_type) != feedback_type
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type | feedback_type)

        trans_id = self._get_trans_id()
        if feedback_key and self._set_feedback_key_tranid:
            self._set_feedback_key_tranid(feedback_key, trans_id, self._feedback_type)
        ret = self.send_modbus_request(funcode, datas, num)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        ret = self.recv_modbus_response(funcode, ret, 0, self._S_TOUT if timeout is None else timeout)
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type)
        return ret

    @lock_require
    def getset_nu8(self, funcode, datas, num_send, num_get):
        ret = self.send_modbus_request(funcode, datas, num_send)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(funcode, ret, num_get, self._S_TOUT)

    @lock_require
    def get_nu8(self, funcode, num):
        ret = self.send_modbus_request(funcode, 0, 0)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num + 1)
        return self.recv_modbus_response(funcode, ret, num, self._G_TOUT)

    @lock_require
    def set_nu16(self, funcode, datas, num, additional_bytes=None):
        hexdata = convert.u16s_to_bytes(datas, num)
        if additional_bytes is not None:
            hexdata += additional_bytes
            ret = self.send_modbus_request(funcode, hexdata, num * 2 + len(additional_bytes))
        else:
            ret = self.send_modbus_request(funcode, hexdata, num * 2)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        ret = self.recv_modbus_response(funcode, ret, 0, self._S_TOUT)
        return ret

    @lock_require
    def get_nu16(self, funcode, num):
        ret = self.send_modbus_request(funcode, 0, 0)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num * 2 + 1)
        ret = self.recv_modbus_response(funcode, ret, num * 2, self._G_TOUT)
        data = [0] * (1 + num)
        data[0] = ret[0]
        data[1:num] = convert.bytes_to_u16s(ret[1:num * 2 + 1], num)
        return data

    @lock_require
    def set_nfp32(self, funcode, datas, num, feedback_key=None, feedback_type=XCONF.FeedbackType.MOTION_FINISH):
        need_set_fb = feedback_type != 0 and (self._feedback_type & feedback_type) != feedback_type
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type | feedback_type)

        trans_id = self._get_trans_id()
        if feedback_key and self._set_feedback_key_tranid:
            self._set_feedback_key_tranid(feedback_key, trans_id, self._feedback_type)
        hexdata = convert.fp32s_to_bytes(datas, num)
        ret = self.send_modbus_request(funcode, hexdata, num * 4)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        ret = self.recv_modbus_response(funcode, ret, 0, self._S_TOUT)
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type)
        return ret

    @lock_require
    def set_nfp32_with_bytes(self, funcode, datas, num, additional_bytes, rx_len=0, timeout=None, feedback_key=None, feedback_type=XCONF.FeedbackType.MOTION_FINISH):
        need_set_fb = feedback_type != 0 and (self._feedback_type & feedback_type) != feedback_type
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type | feedback_type)

        trans_id = self._get_trans_id()
        if feedback_key and self._set_feedback_key_tranid:
            self._set_feedback_key_tranid(feedback_key, trans_id, self._feedback_type)
        hexdata = convert.fp32s_to_bytes(datas, num)
        hexdata += additional_bytes
        ret = self.send_modbus_request(funcode, hexdata, num * 4 + len(additional_bytes))
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        ret = self.recv_modbus_response(funcode, ret, rx_len, self._S_TOUT if timeout is None else timeout)
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type)
        return ret

    @lock_require
    def set_nint32(self, funcode, datas, num, feedback_key=None, feedback_type=XCONF.FeedbackType.MOTION_FINISH):
        need_set_fb = feedback_type != 0 and (self._feedback_type & feedback_type) != feedback_type
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type | feedback_type)

        trans_id = self._get_trans_id()
        if feedback_key and self._set_feedback_key_tranid:
            self._set_feedback_key_tranid(feedback_key, trans_id, self._feedback_type)
        hexdata = convert.int32s_to_bytes(datas, num)
        ret = self.send_modbus_request(funcode, hexdata, num * 4)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        ret = self.recv_modbus_response(funcode, ret, 0, self._S_TOUT)
        if feedback_key and need_set_fb:
            self._set_feedback_type_no_lock(self._feedback_type)
        return ret

    @lock_require
    def get_nfp32(self, funcode, num, timeout=None):
        ret = self.send_modbus_request(funcode, 0, 0)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num * 4 + 1)
        ret = self.recv_modbus_response(funcode, ret, num * 4, timeout if timeout is not None else self._G_TOUT)
        data = [0] * (1 + num)
        data[0] = ret[0]
        data[1:num+1] = convert.bytes_to_fp32s(ret[1:num * 4 + 1], num)
        return data

    @lock_require
    def get_nfp32_with_datas(self, funcode, datas, num_send, num_get, timeout=None):
        ret = self.send_modbus_request(funcode, datas, num_send)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        ret = self.recv_modbus_response(funcode, ret, num_get * 4, timeout if timeout is not None else self._G_TOUT)
        data = [0] * (1 + num_get)
        data[0] = ret[0]
        data[1:num_get + 1] = convert.bytes_to_fp32s(ret[1:num_get * 4 + 1], num_get)
        return data

    @lock_require
    def swop_nfp32(self, funcode, datas, txn, rxn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_modbus_request(funcode, hexdata, txn * 4)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (rxn + 1)
        ret = self.recv_modbus_response(funcode, ret, rxn * 4, self._G_TOUT)
        data = [0] * (1 + rxn)
        data[0] = ret[0]
        data[1:rxn+1] = convert.bytes_to_fp32s(ret[1:rxn * 4 + 1], rxn)
        return data

    @lock_require
    def is_nfp32(self, funcode, datas, txn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_modbus_request(funcode, hexdata, txn * 4)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * 2
        return self.recv_modbus_response(funcode, ret, 1, self._G_TOUT)

    def get_version(self):
        return self.get_nu8(XCONF.UxbusReg.GET_VERSION, 40)

    def get_robot_sn(self):
        return self.get_nu8(XCONF.UxbusReg.GET_ROBOT_SN, 40)

    def check_verification(self):
        # txdata = signature, 175: signature length if use 14-character SN for plain text, do not miss '\n's
        return self.get_nu8(XCONF.UxbusReg.CHECK_VERIFY, 1)

    def system_control(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SYSTEM_CONTROL, txdata, 1)

    def set_record_traj(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_TRAJ_RECORD, txdata, 1)

    def playback_traj(self, value, spdx=1, feedback_key=None):
        txdata = [value, spdx]
        return self.set_nint32(XCONF.UxbusReg.PLAY_TRAJ, txdata, 2, feedback_key=feedback_key, feedback_type=XCONF.FeedbackType.OTHER_FINISH)

    def playback_traj_old(self, value):
        txdata = [value]
        return self.set_nint32(XCONF.UxbusReg.PLAY_TRAJ, txdata, 1)

    def save_traj(self, filename, wait_time=2, feedback_key=None):
        char_list = list(filename)
        txdata = [ord(i) for i in char_list]
        name_len = len(txdata)
        if name_len > 80:
            print("name length should not exceed 80 characters!")
            return [XCONF.UxbusState.ERR_PARAM]
        txdata = txdata + [0] * (81 - name_len)

        ret = self.set_nu8(XCONF.UxbusReg.SAVE_TRAJ, txdata, 81, feedback_key=feedback_key, feedback_type=XCONF.FeedbackType.OTHER_FINISH)
        time.sleep(wait_time)  # Must! or buffer would be flushed if set mode to pos_mode
        return ret

    def load_traj(self, filename, wait_time=2, feedback_key=None):
        char_list = list(filename)
        txdata = [ord(i) for i in char_list]
        name_len = len(txdata)
        if name_len > 80:
            print("name length should not exceed 80 characters!")
            return [XCONF.UxbusState.ERR_PARAM]
        txdata = txdata + [0] * (81 - name_len)

        ret = self.set_nu8(XCONF.UxbusReg.LOAD_TRAJ, txdata, 81, feedback_key=feedback_key, feedback_type=XCONF.FeedbackType.OTHER_FINISH)
        if wait_time > 0:
            time.sleep(wait_time)  # Must! or buffer would be flushed if set mode to pos_mode
        return ret

    def get_traj_rw_status(self):
        return self.get_nu8(XCONF.UxbusReg.GET_TRAJ_RW_STATUS, 1)

    def set_reduced_mode(self, on_off):
        txdata = [on_off]
        return self.set_nu8(XCONF.UxbusReg.SET_REDUCED_MODE, txdata, 1)

    def set_reduced_linespeed(self, lspd_mm):
        txdata = [lspd_mm]
        return self.set_nfp32(XCONF.UxbusReg.SET_REDUCED_TRSV, txdata, 1)

    def set_reduced_jointspeed(self, jspd_rad):
        txdata = [jspd_rad]
        return self.set_nfp32(XCONF.UxbusReg.SET_REDUCED_P2PV, txdata, 1)

    def get_reduced_mode(self):
        return self.get_nu8(XCONF.UxbusReg.GET_REDUCED_MODE, 1)

    def get_reduced_states(self, length=21):
        ret = self.get_nu8(XCONF.UxbusReg.GET_REDUCED_STATE, length)
        msg = [0] * 8
        msg[0] = ret[0]
        msg[1] = ret[1]  # reduced_mode_is_on
        msg[2] = convert.bytes_to_16s(ret[2:14], 6)  # tcp_boundary
        msg[3:5] = convert.bytes_to_fp32s(ret[14:22], 2)  # tcp_speed, joint_speed
        if length == 79:
            msg[5] = convert.bytes_to_fp32s(ret[22:78], 14)  # joint range
            msg[6:8] = ret[78:80]  # fense_is_on, collision_rebound
        return msg

    def set_xyz_limits(self, xyz_list):
        return self.set_nint32(XCONF.UxbusReg.SET_LIMIT_XYZ, xyz_list, 6)

    def set_timer(self, sec_later, timer_id, fun_code, param1=0, param2=0):
        txdata = [sec_later, timer_id, fun_code, param1, param2]
        return self.set_nint32(XCONF.UxbusReg.SET_TIMER, txdata, 5)

    def cancel_timer(self, timer_id):
        txdata = [timer_id]
        return self.set_nint32(XCONF.UxbusReg.CANCEL_TIMER, txdata, 1)

    def set_world_offset(self, pose_offset):
        return self.set_nfp32(XCONF.UxbusReg.SET_WORLD_OFFSET, pose_offset, 6)

    def cnter_reset(self):
        return self.set_nu8(XCONF.UxbusReg.CNTER_RESET, 0, 0)

    def cnter_plus(self):
        return self.set_nu8(XCONF.UxbusReg.CNTER_PLUS, 0, 0)

    def set_reduced_jrange(self, jrange_rad):
        return self.set_nfp32(XCONF.UxbusReg.SET_REDUCED_JRANGE, jrange_rad, 14)

    def set_fense_on(self, on_off):
        txdata = [on_off]
        return self.set_nu8(XCONF.UxbusReg.SET_FENSE_ON, txdata, 1)

    def set_collis_reb(self, on_off):
        txdata = [on_off]
        return self.set_nu8(XCONF.UxbusReg.SET_COLLIS_REB, txdata, 1)

    def motion_en(self, axis_id, enable):
        txdata = [axis_id, int(enable)]
        return self.set_nu8(XCONF.UxbusReg.MOTION_EN, txdata, 2, timeout=self._S_TOUT if self._S_TOUT >= 5 else 5)

    def set_state(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_STATE, txdata, 1)

    def get_state(self):
        return self.get_nu8(XCONF.UxbusReg.GET_STATE, 1)

    def get_cmdnum(self):
        return self.get_nu16(XCONF.UxbusReg.GET_CMDNUM, 1)

    def get_err_code(self):
        return self.get_nu8(XCONF.UxbusReg.GET_ERROR, 2)

    def get_hd_types(self):
        return self.get_nu8(XCONF.UxbusReg.GET_HD_TYPES, 2)

    def reload_dynamics(self):
        return self.set_nu8(XCONF.UxbusReg.RELOAD_DYNAMICS, 0, 0)

    def clean_err(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_ERR, 0, 0)

    def clean_war(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_WAR, 0, 0)

    def set_brake(self, axis_id, enable):
        txdata = [axis_id, int(enable)]
        return self.set_nu8(XCONF.UxbusReg.SET_BRAKE, txdata, 2)

    def set_mode(self, mode, detection_param=-1):
        if detection_param >= 0:
            txdata = [mode, detection_param]
            return self.set_nu8(XCONF.UxbusReg.SET_MODE, txdata, 2)
        else:
            txdata = [mode]
            return self.set_nu8(XCONF.UxbusReg.SET_MODE, txdata, 1)

    def set_report_tau_or_i(self, tau_or_i):  # 0 for tau(default), 1 for i
        txdata = [tau_or_i]
        return self.set_nu8(XCONF.UxbusReg.REPORT_TAU_OR_I, txdata, 1)

    def get_report_tau_or_i(self):
        return self.get_nu8(XCONF.UxbusReg.GET_REPORT_TAU_OR_I, 1)

    def set_cartesian_velo_continuous(self, on_off):  # False for not continuous, True for continuous
        txdata = [int(on_off)]
        return self.set_nu8(XCONF.UxbusReg.SET_CARTV_CONTINUE, txdata, 1)

    def set_allow_approx_motion(self, on_off):
        txdata = [int(on_off)]
        return self.set_nu8(XCONF.UxbusReg.SET_ALLOW_APPROX_MOTION, txdata, 1)

    def get_allow_approx_motion(self):
        return self.get_nu8(XCONF.UxbusReg.GET_ALLOW_APPROX_MOTION, 1)

    def move_line(self, mvpose, mvvelo, mvacc, mvtime, only_check_type=0, motion_type=0):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime]
        if only_check_type <= 0 and motion_type == 0:
            return self.set_nfp32(XCONF.UxbusReg.MOVE_LINE, txdata, 9)
        else:
            byte_data = bytes([only_check_type]) if motion_type == 0 else bytes([only_check_type, int(motion_type)])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_LINE, txdata, 9, byte_data, 3, timeout=10)

    def move_line_common(self, mvpose, mvvelo, mvacc, mvtime, radius=-1, coord=0, is_axis_angle=False, only_check_type=0, motion_type=0, feedback_key=None):
        """
        通用指令, 固件1.10.0开始支持 
        """
        txdata = [mvpose[i] for i in range(6)]
        _radius = -1 if radius is None else radius
        txdata += [mvvelo, mvacc, mvtime, _radius]
        if motion_type == 0:
            byte_data = bytes([coord, int(is_axis_angle), only_check_type])
        else:
            byte_data = bytes([coord, int(is_axis_angle), only_check_type, int(motion_type)])
        return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_LINE, txdata, 10, byte_data, 3, timeout=10, feedback_key=feedback_key)

    def move_line_aa(self, mvpose, mvvelo, mvacc, mvtime, mvcoord, relative, only_check_type=0, motion_type=0):
        float_data = [mvpose[i] for i in range(6)]
        float_data += [mvvelo, mvacc, mvtime]
        byte_data = bytes([mvcoord, relative])
        if only_check_type <= 0 and motion_type == 0:
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_LINE_AA, float_data, 9, byte_data)
        else:
            byte_data += bytes([only_check_type]) if motion_type == 0 else bytes([only_check_type, int(motion_type)])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_LINE_AA, float_data, 9, byte_data, 3, timeout=10)

    def move_servo_cart_aa(self, mvpose, mvvelo, mvacc, tool_coord, relative):
        float_data = [mvpose[i] for i in range(6)]
        float_data += [mvvelo, mvacc, tool_coord]
        byte_data = bytes([relative])
        return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_SERVO_CART_AA, float_data, 9, byte_data)

    def move_relative(self, pose, mvvelo, mvacc, mvtime, radius, is_joint_motion=False, is_angle_axis=False, only_check_type=0, motion_type=0, feedback_key=None):
        float_data = [0] * 7
        for i in range(min(7, len(pose))):
            float_data[i] = pose[i]
        float_data += [mvvelo, mvacc, mvtime, radius]
        byte_data = bytes([int(is_joint_motion), int(is_angle_axis)])
        if only_check_type <= 0 and motion_type == 0:
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_RELATIVE, float_data, 11, byte_data, feedback_key=feedback_key)
        else:
            byte_data += bytes([only_check_type]) if motion_type == 0 else bytes([only_check_type, int(motion_type)])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_RELATIVE, float_data, 11, byte_data, 3, timeout=10, feedback_key=feedback_key)

    def get_position_aa(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_TCP_POSE_AA, 6)

    @lock_require
    def get_pose_offset(self, pose1, pose2, orient_type_in=0, orient_type_out=0):
        float_data = [pose1[i] for i in range(6)]
        float_data += [pose2[j] for j in range(6)]
        byte_data = bytes([orient_type_in, orient_type_out])
        ret_fp_num = 6
        funcode = XCONF.UxbusReg.CAL_POSE_OFFSET
        hexdata = convert.fp32s_to_bytes(float_data, 12)
        hexdata += byte_data

        ret = self.send_modbus_request(funcode, hexdata, len(hexdata))
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (ret_fp_num * 4 + 1)

        ret = self.recv_modbus_response(funcode, ret, ret_fp_num * 4, self._G_TOUT)
        data = [0] * (1 + ret_fp_num)
        data[0] = ret[0]
        data[1:ret_fp_num+1] = convert.bytes_to_fp32s(ret[1:ret_fp_num * 4 + 1], ret_fp_num)
        return data

    def move_line_tool(self, mvpose, mvvelo, mvacc, mvtime, only_check_type=0, motion_type=0):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime]
        if only_check_type <= 0 and motion_type == 0:
            return self.set_nfp32(XCONF.UxbusReg.MOVE_LINE_TOOL, txdata, 9)
        else:
            byte_data = bytes([only_check_type]) if motion_type == 0 else bytes([only_check_type, int(motion_type)])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_LINE_TOOL, txdata, 9, byte_data, 3, timeout=10)

    def move_lineb(self, mvpose, mvvelo, mvacc, mvtime, mvradii, only_check_type=0, motion_type=0):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime, mvradii]
        if only_check_type <= 0 and motion_type == 0:
            return self.set_nfp32(XCONF.UxbusReg.MOVE_LINEB, txdata, 10)
        else:
            byte_data = bytes([only_check_type]) if motion_type == 0 else bytes([only_check_type, int(motion_type)])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_LINEB, txdata, 10, byte_data, 3, timeout=10)

    def move_joint(self, mvjoint, mvvelo, mvacc, mvtime, only_check_type=0, feedback_key=None):
        txdata = [mvjoint[i] for i in range(7)]
        txdata += [mvvelo, mvacc, mvtime]
        if only_check_type <= 0:
            return self.set_nfp32(XCONF.UxbusReg.MOVE_JOINT, txdata, 10, feedback_key=feedback_key)
        else:
            byte_data = bytes([only_check_type])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_JOINT, txdata, 10, byte_data, 3, timeout=10, feedback_key=feedback_key)

    def move_jointb(self, mvjoint, mvvelo, mvacc, mvradii, only_check_type=0, feedback_key=None):
        txdata = [mvjoint[i] for i in range(7)]
        txdata += [mvvelo, mvacc, mvradii]
        if only_check_type <= 0:
            return self.set_nfp32(XCONF.UxbusReg.MOVE_JOINTB, txdata, 10, feedback_key=feedback_key)
        else:
            byte_data = bytes([only_check_type])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_JOINTB, txdata, 10, byte_data, 3, timeout=10, feedback_key=feedback_key)

    def move_gohome(self, mvvelo, mvacc, mvtime, only_check_type=0, feedback_key=None):
        txdata = [mvvelo, mvacc, mvtime]
        if only_check_type <= 0:
            return self.set_nfp32(XCONF.UxbusReg.MOVE_HOME, txdata, 3, feedback_key=feedback_key)
        else:
            byte_data = bytes([only_check_type])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_HOME, txdata, 3, byte_data, 3, timeout=10, feedback_key=feedback_key)

    def move_servoj(self, mvjoint, mvvelo, mvacc, mvtime):
        txdata = [mvjoint[i] for i in range(7)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_SERVOJ, txdata, 10)

    def move_servo_cartesian(self, mvpose, mvvelo, mvacc, mvtime):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_SERVO_CART, txdata, 9)

    # # This interface is no longer supported
    # def set_servot(self, jnt_taus):
    #     txdata = [jnt_taus[i] for i in range(7)]
    #     return self.set_nfp32(XCONF.UxbusReg.SET_SERVOT, txdata, 7)

    def get_joint_tau(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_JOINT_TAU, 7)

    def set_safe_level(self, level):
        txdata = [level]
        return self.set_nu8(XCONF.UxbusReg.SET_SAFE_LEVEL, txdata, 1)

    def get_safe_level(self):
        return self.get_nu8(XCONF.UxbusReg.GET_SAFE_LEVEL, 1)

    def sleep_instruction(self, sltime):
        txdata = [sltime]
        return self.set_nfp32(XCONF.UxbusReg.SLEEP_INSTT, txdata, 1)

    def move_circle(self, pose1, pose2, mvvelo, mvacc, mvtime, percent, only_check_type=0):
        txdata = [0] * 16
        for i in range(6):
            txdata[i] = pose1[i]
            txdata[6 + i] = pose2[i]
        txdata[12] = mvvelo
        txdata[13] = mvacc
        txdata[14] = mvtime
        txdata[15] = percent
        if only_check_type <= 0:
            return self.set_nfp32(XCONF.UxbusReg.MOVE_CIRCLE, txdata, 16)
        else:
            byte_data = bytes([only_check_type])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_CIRCLE, txdata, 16, byte_data, 3, timeout=10)

    def move_circle_common(self, pose1, pose2, mvvelo, mvacc, mvtime, percent, coord=0, is_axis_angle=False, only_check_type=0, feedback_key=None):
        """
        通用指令, 固件1.10.0开始支持 
        """
        txdata = [0] * 16
        for i in range(6):
            txdata[i] = pose1[i]
            txdata[6 + i] = pose2[i]
        txdata[12] = mvvelo
        txdata[13] = mvacc
        txdata[14] = mvtime
        txdata[15] = percent
        byte_data = bytes([coord, int(is_axis_angle), only_check_type])
        return self.set_nfp32_with_bytes(XCONF.UxbusReg.MOVE_CIRCLE, txdata, 16, byte_data, 3, timeout=10, feedback_key=feedback_key)

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

    def set_tcp_load(self, load_mass, load_com, feedback_key=None):
        param_list = [load_mass]
        param_list.extend(load_com)
        return self.set_nfp32(XCONF.UxbusReg.SET_LOAD_PARAM, param_list, 4, feedback_key=feedback_key, feedback_type=XCONF.FeedbackType.TRIGGER)

    def set_collis_sens(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_COLLIS_SENS, txdata, 1)

    def set_teach_sens(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_TEACH_SENS, txdata, 1)

    def set_gravity_dir(self, gravity_dir):
        return self.set_nfp32(XCONF.UxbusReg.SET_GRAVITY_DIR, gravity_dir, 3)

    def clean_conf(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_CONF, 0, 0)

    def save_conf(self):
        return self.set_nu8(XCONF.UxbusReg.SAVE_CONF, 0, 0)

    def get_joint_pos(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_JOINT_POS, 7)

    def get_joint_states(self, num=3):
        return self.get_nfp32_with_datas(XCONF.UxbusReg.GET_JOINT_POS, [num], 1, 7 * num)

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

    @lock_require
    def gripper_addr_w16(self, addr, value):
        return self.tgpio_addr_w16(addr, value, bid=XCONF.GRIPPER_ID)

    @lock_require
    def gripper_addr_r16(self, addr):
        return self.tgpio_addr_r16(addr, bid=XCONF.GRIPPER_ID)

    @lock_require
    def gripper_addr_w32(self, addr, value):
        return self.tgpio_addr_w32(addr, value, bid=XCONF.GRIPPER_ID)

    @lock_require
    def gripper_addr_r32(self, addr):
        return self.tgpio_addr_r32(addr, bid=XCONF.GRIPPER_ID)

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
        ret = self.get_nu8(XCONF.UxbusReg.TGPIO_ERR, 2)
        return ret

    def gripper_clean_err(self):
        return self.gripper_addr_w16(XCONF.ServoConf.RESET_ERR, 1)

    @lock_require
    def tgpio_addr_w16(self, addr, value, bid=XCONF.TGPIO_HOST_ID, additional_bytes=None):
        txdata = bytes([bid])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        if additional_bytes is not None:
            txdata += additional_bytes
            ret = self.send_modbus_request(XCONF.UxbusReg.TGPIO_W16B, txdata, 7 + len(additional_bytes))
        else:
            ret = self.send_modbus_request(XCONF.UxbusReg.TGPIO_W16B, txdata, 7)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.TGPIO_W16B, ret, 0, self._G_TOUT)
        return ret

    @lock_require
    def tgpio_addr_r16(self, addr, bid=XCONF.TGPIO_HOST_ID, fmt='>l'):
        txdata = bytes([bid])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_modbus_request(XCONF.UxbusReg.TGPIO_R16B, txdata, 3)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.TGPIO_R16B, ret, 4, self._G_TOUT)
        return [ret[0], convert.bytes_to_num32(ret[1:5], fmt=fmt)]

    @lock_require
    def tgpio_addr_w32(self, addr, value, bid=XCONF.TGPIO_HOST_ID):
        txdata = bytes([bid])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_modbus_request(XCONF.UxbusReg.TGPIO_W32B, txdata, 7)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.TGPIO_W32B, ret, 0, self._G_TOUT)
        return ret

    @lock_require
    def tgpio_addr_r32(self, addr, bid=XCONF.TGPIO_HOST_ID, fmt='>l'):
        txdata = bytes([bid])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_modbus_request(XCONF.UxbusReg.TGPIO_R32B, txdata, 3)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.TGPIO_R32B, ret, 4, self._G_TOUT)
        return [ret[0], convert.bytes_to_num32(ret[1:5], fmt=fmt)]

    def tgpio_get_digital(self):
        ret = self.tgpio_addr_r16(XCONF.ServoConf.DIGITAL_IN)
        value = [0] * 5
        value[0] = ret[0]
        value[1] = ret[1] & 0x0001
        value[2] = (ret[1] & 0x0002) >> 1
        value[3] = (ret[1] & 0x0004) >> 2
        value[4] = (ret[1] & 0x0008) >> 3
        return value

    def tgpio_set_digital(self, ionum, value, sync=None):
        tmp = 0
        if ionum == 1:
            tmp = tmp | 0x0100
            if value:
                tmp = tmp | 0x0001
        elif ionum == 2:
            tmp = tmp | 0x0200
            if value:
                tmp = tmp | 0x0002
        elif ionum == 3:
            tmp = tmp | 0x1000
            if value:
                tmp = tmp | 0x0010
        elif ionum == 4:
            tmp = tmp | 0x0400
            if value:
                tmp = tmp | 0x0004
        elif ionum == 5:
            tmp = tmp | 0x0800
            if value:
                tmp = tmp | 0x0008
        else:
            return [-1, -1]
        if sync is not None:
            return self.tgpio_addr_w16(XCONF.ServoConf.DIGITAL_OUT, tmp, additional_bytes=bytes([sync]))
        else:
            return self.tgpio_addr_w16(XCONF.ServoConf.DIGITAL_OUT, tmp)        

    def tgpio_get_analog1(self):
        ret = self.tgpio_addr_r16(XCONF.ServoConf.ANALOG_IO1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 3.3 / 4095.0
        return value

    def tgpio_get_analog2(self):
        ret = self.tgpio_addr_r16(XCONF.ServoConf.ANALOG_IO2)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 3.3 / 4095.0
        return value

    def set_modbus_timeout(self, value, is_transparent_transmission=False):
        txdata = [int(value)]
        return self.set_nu16(XCONF.UxbusReg.TGPIO_COM_TIOUT if is_transparent_transmission else XCONF.UxbusReg.TGPIO_MB_TIOUT, txdata, 1)

    def set_modbus_baudrate(self, baudrate):
        if baudrate not in self.BAUDRATES:
            return [-1, -1]
        ret = self.tgpio_addr_r16(XCONF.ServoConf.MODBUS_BAUDRATE & 0x0FFF)
        if ret[0] == 0:
            baud_val = self.BAUDRATES.index(baudrate)
            if ret[1] != baud_val:
                # self.tgpio_addr_w16(XCONF.ServoConf.MODBUS_BAUDRATE, baud_val)
                self.tgpio_addr_w16(0x1A0B, baud_val)
                time.sleep(0.3)
                return self.tgpio_addr_w16(XCONF.ServoConf.SOFT_REBOOT, 1)
        return ret[:2]

    @lock_require
    def tgpio_set_modbus(self, modbus_t, len_t, host_id=XCONF.TGPIO_HOST_ID, limit_sec=0.0, is_transparent_transmission=False):
        txdata = bytes([host_id])
        txdata += bytes(modbus_t)
        if limit_sec > 0:
            diff_time = time.monotonic() - self._last_modbus_comm_time
            if diff_time < limit_sec:
                time.sleep(limit_sec - diff_time)
        ret = self.send_modbus_request(XCONF.UxbusReg.TGPIO_COM_DATA if is_transparent_transmission else XCONF.UxbusReg.TGPIO_MODBUS, txdata, len_t + 1)
        if ret == -1:
            self._last_modbus_comm_time = time.monotonic()
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.TGPIO_COM_DATA if is_transparent_transmission else XCONF.UxbusReg.TGPIO_MODBUS, ret, -1, self._G_TOUT)
        self._last_modbus_comm_time = time.monotonic()
        return ret

    @lock_require
    def tgpio_delay_set_digital(self, ionum, on_off, delay_sec):
        txdata = bytes([ionum, on_off])
        txdata += convert.fp32_to_bytes(delay_sec)
        ret = self.send_modbus_request(XCONF.UxbusReg.DELAYED_TGPIO_SET, txdata, 6)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.DELAYED_TGPIO_SET, ret, 0, self._S_TOUT)

    @lock_require
    def cgpio_delay_set_digital(self, ionum, on_off, delay_sec):
        txdata = bytes([ionum, on_off])
        txdata += convert.fp32_to_bytes(delay_sec)
        ret = self.send_modbus_request(XCONF.UxbusReg.DELAYED_CGPIO_SET, txdata, 6)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.DELAYED_CGPIO_SET, ret, 0, self._S_TOUT)

    @lock_require
    def cgpio_position_set_digital(self, ionum, on_off, xyz, tol_r):
        txdata = bytes([ionum, on_off])
        txdata += convert.fp32s_to_bytes(xyz, 3)
        txdata += convert.fp32_to_bytes(tol_r)
        ret = self.send_modbus_request(XCONF.UxbusReg.POSITION_CGPIO_SET, txdata, 18)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.POSITION_CGPIO_SET, ret, 0, self._S_TOUT)

    @lock_require
    def tgpio_position_set_digital(self, ionum, on_off, xyz, tol_r):
        txdata = bytes([ionum, on_off])
        txdata += convert.fp32s_to_bytes(xyz, 3)
        txdata += convert.fp32_to_bytes(tol_r)
        ret = self.send_modbus_request(XCONF.UxbusReg.POSITION_TGPIO_SET, txdata, 18)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.POSITION_TGPIO_SET, ret, 0, self._S_TOUT)

    @lock_require
    def cgpio_position_set_analog(self, ionum, value, xyz, tol_r):
        txdata = bytes([ionum])
        txdata += convert.u16_to_bytes(int(value / 10.0 * 4095.0))
        txdata += convert.fp32s_to_bytes(xyz, 3)
        txdata += convert.fp32_to_bytes(tol_r)
        ret = self.send_modbus_request(XCONF.UxbusReg.POSITION_CGPIO_SET_ANALOG, txdata, 19)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.POSITION_CGPIO_SET_ANALOG, ret, 0, self._S_TOUT)

    # io_type: 0 for CGPIO, 1 for TGPIO
    def config_io_stop_reset(self, io_type, on_off):
        txdata = [io_type, on_off]
        return self.set_nu8(XCONF.UxbusReg.SET_IO_STOP_RESET, txdata, 2)

    def gripper_modbus_w16s(self, addr, value, length):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += bytes([0x10])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.u16_to_bytes(length)
        txdata += bytes([length * 2])
        txdata += value
        ret = self.tgpio_set_modbus(txdata, length * 2 + 7)
        return ret

    def gripper_modbus_r16s(self, addr, length):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += bytes([0x03])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.u16_to_bytes(length)
        ret = self.tgpio_set_modbus(txdata, 6)
        return ret

    def gripper_modbus_set_en(self, value):
        value = convert.u16_to_bytes(int(value))
        return self.gripper_modbus_w16s(XCONF.ServoConf.CON_EN, value, 1)

    def gripper_modbus_set_mode(self, value):
        value = convert.u16_to_bytes(int(value))
        return self.gripper_modbus_w16s(XCONF.ServoConf.CON_MODE, value, 1)

    def gripper_modbus_set_zero(self):
        value = convert.u16_to_bytes(int(1))
        return self.gripper_modbus_w16s(XCONF.ServoConf.MT_ZERO, value, 1)

    def gripper_modbus_get_pos(self):
        ret = self.gripper_modbus_r16s(XCONF.ServoConf.CURR_POS, 2)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) == 9:
            ret1[1] = convert.bytes_to_long_big(ret[5:9])
        else:
            if ret1[0] == 0:
                ret1[0] = XCONF.UxbusState.ERR_LENG
            # print('gripper_modbus_get_pos:', len(ret), ret)
        # print(ret1, ret)
        return ret1

    def gripper_modbus_set_pos(self, pulse):
        value = bytes([(int(pulse) >> 24) & 0xFF])
        value += bytes([(int(pulse) >> 16) & 0xFF])
        value += bytes([(int(pulse) >> 8) & 0xFF])
        value += bytes([int(pulse) & 0xFF])
        return self.gripper_modbus_w16s(XCONF.ServoConf.TAGET_POS, value, 2)

    def gripper_modbus_set_posspd(self, speed):
        speed = convert.u16_to_bytes(int(speed))
        return self.gripper_modbus_w16s(XCONF.ServoConf.POS_SPD, speed, 1)

    def gripper_modbus_get_errcode(self):
        ret = self.gripper_modbus_r16s(XCONF.ServoConf.ERR_CODE, 1)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) == 7:
            ret1[1] = convert.bytes_to_u16(ret[5:7])
        else:
            if ret1[0] == 0:
                ret1[0] = XCONF.UxbusState.ERR_LENG
            # print('gripper_modbus_get_errcode:', len(ret), ret)
        # print(ret1, ret)
        return ret1

    def gripper_modbus_clean_err(self):
        value = convert.u16_to_bytes(int(1))
        return self.gripper_modbus_w16s(XCONF.ServoConf.RESET_ERR, value, 1)

    def servo_set_zero(self, axis_id):
        txdata = [int(axis_id)]
        ret = self.set_nu8(XCONF.UxbusReg.SERVO_ZERO, txdata, 1)
        return ret

    def servo_get_dbmsg(self):
        ret = self.get_nu8(XCONF.UxbusReg.SERVO_DBMSG, 16)
        return ret

    @lock_require
    def servo_addr_w16(self, axis_id, addr, value):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_modbus_request(XCONF.UxbusReg.SERVO_W16B, txdata, 7)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.SERVO_W16B, ret, 0, self._G_TOUT)
        return ret

    @lock_require
    def servo_addr_r16(self, axis_id, addr):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_modbus_request(XCONF.UxbusReg.SERVO_R16B, txdata, 3)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.SERVO_R16B, ret, 4, self._G_TOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]
        # return [ret[0], convert.bytes_to_long_big(ret[1:5])[0]]

    @lock_require
    def servo_addr_w32(self, axis_id, addr, value):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_modbus_request(XCONF.UxbusReg.SERVO_W32B, txdata, 7)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.SERVO_W32B, ret, 0, self._G_TOUT)
        return ret

    @lock_require
    def servo_addr_r32(self, axis, addr):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_modbus_request(XCONF.UxbusReg.SERVO_R32B, txdata, 3)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.SERVO_R32B, ret, 4, self._G_TOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]
        # return [ret[0], convert.bytes_to_long_big(ret[1:5])[0]]

    # -----------------------------------------------------
    # controler gpio
    # -----------------------------------------------------
    def cgpio_get_auxdigit(self):
        ret = self.get_nu16(XCONF.UxbusReg.CGPIO_GET_DIGIT, 1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1]
        return value

    def cgpio_get_analog1(self):
        ret = self.get_nu16(XCONF.UxbusReg.CGPIO_GET_ANALOG1, 1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 10.0 / 4095.0
        return value

    def cgpio_get_analog2(self):
        ret = self.get_nu16(XCONF.UxbusReg.CGPIO_GET_ANALOG2, 1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 10.0 / 4095.0
        return value

    def cgpio_set_auxdigit(self, ionum, value, sync=None):
        tmp = [0] * 2
        if ionum > 7:
            tmp[1] = tmp[1] | (0x0100 << (ionum - 8))
            if value:
                tmp[1] = tmp[1] | (0x0001 << (ionum - 8))
            if sync is not None:
                return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_DIGIT, tmp, 2, additional_bytes=bytes([sync]))
            else:
                return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_DIGIT, tmp, 2)
        else:
            tmp[0] = tmp[0] | (0x0100 << ionum)
            if value:
                tmp[0] = tmp[0] | (0x0001 << ionum)
            if sync is not None:
                return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_DIGIT, tmp, 1, additional_bytes=bytes([sync]))
            else:
                return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_DIGIT, tmp, 1)
        # return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_DIGIT, tmp, 2 if ionum > 7 else 1)

    def cgpio_set_analog1(self, value, sync=None):
        txdata = [int(value / 10.0 * 4095.0)]
        if sync is not None:
            return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_ANALOG1, txdata, 1, additional_bytes=bytes([sync]))
        else:
            return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_ANALOG1, txdata, 1)

    def cgpio_set_analog2(self, value, sync=None):
        txdata = [int(value / 10.0 * 4095.0)]
        if sync is not None:
            return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_ANALOG2, txdata, 1, additional_bytes=bytes([sync]))
        else:
            return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_ANALOG2, txdata, 1)

    def cgpio_set_infun(self, num, fun):
        txdata = [int(num), int(fun)]
        return self.set_nu8(XCONF.UxbusReg.CGPIO_SET_IN_FUN, txdata, 2)

    def cgpio_set_outfun(self, num, fun):
        txdata = [int(num), int(fun)]
        return self.set_nu8(XCONF.UxbusReg.CGPIO_SET_OUT_FUN, txdata, 2)

    def cgpio_get_state(self):
        # ret = self.get_nu8(XCONF.UxbusReg.CGPIO_GET_STATE, 34)
        # ret = self.get_nu8(XCONF.UxbusReg.CGPIO_GET_STATE, 50)
        ret = self.get_nu8(XCONF.UxbusReg.CGPIO_GET_STATE, -1)
        msg = [0] * 13
        msg[0] = ret[0]
        msg[1] = ret[1]
        msg[2] = ret[2]

        msg[3:11] = convert.bytes_to_u16s(ret[3:19], 8)
        msg[7] = msg[7] / 4095.0 * 10.0
        msg[8] = msg[8] / 4095.0 * 10.0
        msg[9] = msg[9] / 4095.0 * 10.0
        msg[10] = msg[10] / 4095.0 * 10.0
        msg[11] = ret[19:27]
        msg[12] = ret[27:35]
        if len(ret) >= 50:
            msg[11] = ret[19:27] + ret[35:43]
            msg[12] = ret[27:35] + ret[43:51]
        return msg

    def set_self_collision_detection(self, on_off):
        txdata = [on_off]
        return self.set_nu8(XCONF.UxbusReg.SET_SELF_COLLIS_CHECK, txdata, 1)

    def set_collision_tool_model(self, tool_type, params):
        if len(params) > 0:
            byte_data = bytes([tool_type])
            return self.set_nfp32_with_bytes(XCONF.UxbusReg.SET_COLLIS_TOOL, params, len(params), byte_data)
        else:
            txdata = [tool_type]
            return self.set_nu8(XCONF.UxbusReg.SET_COLLIS_TOOL, txdata, 1)

    def set_simulation_robot(self, on_off):
        txdata = [int(on_off)]
        return self.set_nu8(XCONF.UxbusReg.SET_SIMULATION_ROBOT, txdata, 1)

    def get_power_board_version(self):
        return self.get_nu8(XCONF.UxbusReg.GET_PWR_VERSION, 6)
    
    def get_movement(self):
        return self.get_nu8(XCONF.UxbusReg.GET_MOVEMENT, 1)
    
    def vc_set_jointv(self, jnt_v, jnt_sync, duration=-1):
        additional_bytes = bytes([jnt_sync])
        if duration >= 0:
            additional_bytes += convert.fp32_to_bytes(duration)
        return self.set_nfp32_with_bytes(XCONF.UxbusReg.VC_SET_JOINTV, jnt_v, 7, additional_bytes)

    def vc_set_linev(self, line_v, coord, duration=-1):
        additional_bytes = bytes([coord])
        if duration >= 0:
            additional_bytes += convert.fp32_to_bytes(duration)
        return self.set_nfp32_with_bytes(XCONF.UxbusReg.VC_SET_CARTV, line_v, 6, additional_bytes)

    def iden_load(self, iden_type, num_get, timeout=500, estimated_mass=0):
        txdata = bytes([iden_type])
        if estimated_mass > 0:
            txdata += convert.fp32_to_bytes(estimated_mass)
        return self.get_nfp32_with_datas(XCONF.UxbusReg.IDEN_LOAD, txdata, 5 if estimated_mass > 0 else 1, num_get, timeout=timeout)

    def iden_joint_friction(self, sn, timeout=500):
        txdata = [ord(i) for i in list(sn)]
        return self.get_nfp32_with_datas(XCONF.UxbusReg.IDEN_FRIC, txdata, 14, 1, timeout=timeout)

    @lock_require
    def set_impedance(self, coord, c_axis, M, K, B):
        txdata = bytes([coord])
        txdata += bytes(c_axis[:6])
        txdata += convert.fp32s_to_bytes(M, 6)
        txdata += convert.fp32s_to_bytes(K, 6)
        txdata += convert.fp32s_to_bytes(B, 6)
        ret = self.send_modbus_request(XCONF.UxbusReg.IMPEDANCE_CONFIG, txdata, 79)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.IMPEDANCE_CONFIG, ret, 0, self._S_TOUT)

    @lock_require
    def set_impedance_mbk(self, M, K, B):
        txdata = convert.fp32s_to_bytes(M, 6)
        txdata += convert.fp32s_to_bytes(K, 6)
        txdata += convert.fp32s_to_bytes(B, 6)
        ret = self.send_modbus_request(XCONF.UxbusReg.IMPEDANCE_CTRL_MBK, txdata, 72)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.IMPEDANCE_CTRL_MBK, ret, 0, self._S_TOUT)

    @lock_require
    def set_impedance_config(self, coord, c_axis):
        txdata = bytes([coord])
        txdata += bytes(c_axis[:6])
        ret = self.send_modbus_request(XCONF.UxbusReg.IMPEDANCE_CTRL_CONFIG, txdata, 7)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.IMPEDANCE_CTRL_CONFIG, ret, 0, self._S_TOUT)

    @lock_require
    def config_force_control(self, coord, c_axis, f_ref, limits):
        txdata = bytes([coord])
        txdata += bytes(c_axis[:6])
        txdata += convert.fp32s_to_bytes(f_ref, 6)
        txdata += convert.fp32s_to_bytes(limits, 6)
        ret = self.send_modbus_request(XCONF.UxbusReg.FORCE_CTRL_CONFIG, txdata, 55)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.FORCE_CTRL_CONFIG, ret, 0, self._S_TOUT)

    @lock_require
    def set_force_control_pid(self, kp, ki, kd, xe_limit):
        txdata = convert.fp32s_to_bytes(kp, 6)
        txdata += convert.fp32s_to_bytes(ki, 6)
        txdata += convert.fp32s_to_bytes(kd, 6)
        txdata += convert.fp32s_to_bytes(xe_limit, 6)
        ret = self.send_modbus_request(XCONF.UxbusReg.FORCE_CTRL_PID, txdata, 96)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.FORCE_CTRL_PID, ret, 0, self._S_TOUT)

    def ft_sensor_set_zero(self):
        return self.set_nu8(XCONF.UxbusReg.FTSENSOR_SET_ZERO, 0, 0)
        # return self.getset_nu8(XCONF.UxbusReg.FTSENSOR_SET_ZERO, [], 0, 1)

    def ft_sensor_iden_load(self):
        return self.iden_load(0, 10)

    def ft_sensor_cali_load(self, iden_result_list):
        return self.set_nfp32(XCONF.UxbusReg.FTSENSOR_CALI_LOAD_OFFSET, iden_result_list, 10)

    def ft_sensor_enable(self, on_off):
        txdata = [on_off]
        return self.set_nu8(XCONF.UxbusReg.FTSENSOR_ENABLE, txdata, 1)

    def ft_sensor_app_set(self, app_code):
        txdata = [app_code]
        return self.set_nu8(XCONF.UxbusReg.FTSENSOR_SET_APP, txdata, 1)

    def ft_sensor_app_get(self):
        return self.get_nu8(XCONF.UxbusReg.FTSENSOR_GET_APP, 1)

    def ft_sensor_get_data(self, is_new=True):
        return self.get_nfp32(XCONF.UxbusReg.FTSENSOR_GET_DATA if is_new else XCONF.UxbusReg.FTSENSOR_GET_DATA_OLD, 6)

    def ft_sensor_get_config(self):
        ret = self.get_nu8(XCONF.UxbusReg.FTSENSOR_GET_CONFIG, 280)
        if ret[0] in [0, 1, 2]:
            ft_app_status = ret[1]
            ft_started = ret[2]
            ft_type = ret[3]
            ft_id = ret[4]
            ft_freq = convert.bytes_to_u16(ret[5:7])
            ft_mass = convert.bytes_to_fp32(ret[7:11])
            ft_dir_bias = convert.bytes_to_fp32(ret[11:15])
            ft_centroid = convert.bytes_to_fp32s(ret[15:27], 3)
            ft_zero = convert.bytes_to_fp32s(ret[27:51], 6)

            imp_coord = ret[51]
            imp_c_axis = ret[52:58]
            M = convert.bytes_to_fp32s(ret[58:82], 6)
            K = convert.bytes_to_fp32s(ret[82:106], 6)
            B = convert.bytes_to_fp32s(ret[106:130], 6)

            fc_coord = ret[130]
            fc_c_axis = ret[131:137]
            force_ref = convert.bytes_to_fp32s(ret[137:161], 6)
            limits = convert.bytes_to_fp32s(ret[161:185], 6)
            kp = convert.bytes_to_fp32s(ret[185:209], 6)
            ki = convert.bytes_to_fp32s(ret[209:233], 6)
            kd = convert.bytes_to_fp32s(ret[233:257], 6)
            xe_limit = convert.bytes_to_fp32s(ret[257:281], 6)
            return [
                ret[0],
                ft_app_status, ft_started, ft_type, ft_id, ft_freq,
                ft_mass, ft_dir_bias, ft_centroid, ft_zero,
                imp_coord, imp_c_axis, M, K, B,
                fc_coord, fc_c_axis, force_ref, limits,
                kp, ki, kd, xe_limit
            ]
        return ret

    def ft_sensor_get_error(self):
        ret = self.servo_addr_r16(8, 0x0010)
        if ret[0] in [0, 1, 2] and len(ret) > 1  and ret[1] == 27:
            ret[0] = 0
        return ret

    # @lock_require
    # def ft_sensor_get_error(self):
    #     txdata = bytes([8])
    #     txdata += convert.u16_to_bytes(0x0010)
    #     ret = self.send_modbus_request(XCONF.UxbusReg.SERVO_R16B, txdata, 3)
    #     if ret == -1:
    #         return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

    #     ret = self.recv_modbus_response(XCONF.UxbusReg.SERVO_R16B, ret, 4, self._G_TOUT)
    #     if ret[0] in [0, 1, 2]:
    #         if convert.bytes_to_long_big(ret[1:5]) == 27:
    #             return [ret[0], 0]
    #         else:
    #             return [ret[0], ret[3]]
    #     return [ret[0], 0]

    def cali_tcp_pose(self, four_pnts):
        txdata = []
        for k in range(4):
            txdata += [four_pnts[k][i] for i in range(6)]
        return self.swop_nfp32(XCONF.UxbusReg.CALI_TCP_POSE, txdata, 24, 3)

    # default: mode: x+ then y+; trust_ind: trust x+ dir
    def cali_user_orient(self, three_pnts, mode=0, trust_ind=0):
        txdata = []
        for k in range(3):
            txdata += [three_pnts[k][i] for i in range(6)]
        byte_data = bytes([mode, trust_ind])
        rxn = 3
        ret = self.set_nfp32_with_bytes(XCONF.UxbusReg.CALI_WRLD_ORIENT, txdata, 18, byte_data, rxn * 4)
        data = [0] * (1 + rxn)
        data[0] = ret[0]
        data[1:rxn+1] = convert.bytes_to_fp32s(ret[1:rxn * 4 + 1], rxn)
        return data

    def cali_tcp_orient(self, rpy_be, rpy_bt):
        txdata = [rpy_be[i] for i in range(3)]
        txdata += [rpy_bt[i] for i in range(3)]
        return self.swop_nfp32(XCONF.UxbusReg.CALI_TCP_ORIENT, txdata, 6, 3)

    def cali_user_pos(self, rpy_ub, pos_b_uorg):
        txdata = [rpy_ub[i] for i in range(3)]
        txdata += [pos_b_uorg[i] for i in range(3)]
        return self.swop_nfp32(XCONF.UxbusReg.CALI_WRLD_POSE, txdata, 6, 3)

    def get_tcp_rotation_radius(self, value):
        txdata = [value]
        data = [0] * 2
        ret = self.getset_nu8(XCONF.UxbusReg.GET_TCP_ROTATION_RADIUS, txdata, 1, 4)
        data[0] = ret[0]
        data[1] = convert.bytes_to_fp32s(ret[1:], 1)
        return data

    def get_max_joint_velocity(self, eveloc, joint_pos):
        txdata = [eveloc]
        txdata += [joint_pos[i] for i in range(7)]
        return self.swop_nfp32(XCONF.UxbusReg.GET_MAX_JOINT_VELOCITY, txdata, 8, 1)

    def track_modbus_w16s(self, addr, value, length):
        txdata = bytes([XCONF.TRACK_ID])
        txdata += bytes([0x10])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.u16_to_bytes(length)
        txdata += bytes([length * 2])
        txdata += value
        ret = self.tgpio_set_modbus(txdata, length * 2 + 7, host_id=XCONF.LINEER_TRACK_HOST_ID, limit_sec=0.001)
        return ret

    def track_modbus_r16s(self, addr, length, fcode=0x03):
        txdata = bytes([XCONF.TRACK_ID])
        txdata += bytes([fcode])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.u16_to_bytes(length)
        ret = self.tgpio_set_modbus(txdata, 6, host_id=XCONF.LINEER_TRACK_HOST_ID, limit_sec=0.001)
        return ret

    def iden_tcp_load(self, estimated_mass=0):
        return self.iden_load(1, 4, timeout=300, estimated_mass=estimated_mass)

    @lock_require
    def servo_error_addr_r32(self, axis, addr):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_modbus_request(XCONF.UxbusReg.SERVO_ERROR, txdata, 3)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.recv_modbus_response(XCONF.UxbusReg.SERVO_ERROR, ret, 4, self._G_TOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]

    def get_dh_params(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_DH, 28)
    
    def set_dh_params(self, dh_params, flag=0):
        if len(dh_params) < 28:
            dh_params.extend([0] * 28 - len(dh_params))
        byte_data = bytes([flag])
        return self.set_nfp32_with_bytes(XCONF.UxbusReg.SET_DH, dh_params, 28, byte_data, 1, timeout=10)

    def _set_feedback_type_no_lock(self, feedback_type):
        ret = self.send_modbus_request(XCONF.UxbusReg.SET_FEEDBACK_TYPE, [feedback_type], 1)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(XCONF.UxbusReg.SET_FEEDBACK_TYPE, ret, 0, self._S_TOUT)

    @lock_require
    def set_feedback_type(self, feedback_type):
        ret = self._set_feedback_type_no_lock(feedback_type)
        if ret[0] != XCONF.UxbusState.ERR_NOTTCP:
            self._feedback_type = feedback_type
        return ret
    
    def check_feedback(self, feedback_key=None):
        ret = self.set_nu8(XCONF.UxbusReg.FEEDBACK_CHECK, [], 0, feedback_key=feedback_key, feedback_type=XCONF.FeedbackType.MOTION_FINISH)
        return ret
    
    @lock_require
    def send_hex_cmd(self, datas, timeout=10):
        if len(datas) < 7:
            # datas length error
            return [-2]
        trans_id = int('{}{}'.format(datas[0], datas[1]), base=16)
        prot_id = int('{}{}'.format(datas[2], datas[3]), base=16)
        if prot_id not in [0, 2, 3]:
            # protocol_identifier error, only support 0/2/3, 
            #   0: standard modbus protocol
            #   2: private modbus protocol
            #   3: private modbus protocol (with heart beat)
            return [-3]
        length = int('{}{}'.format(datas[4], datas[5]), base=16)
        if length != len(datas) - 6:
            # protocol length data error
            return [-4]
        unit_id = int('{}'.format(datas[6]), base=16)
        pdu_data = bytes.fromhex('{}'.format(''.join(map(str, datas[7:]))))
        ret = self.send_modbus_request(unit_id, pdu_data, len(pdu_data), prot_id=prot_id, t_id=trans_id)
        if ret == -1:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.recv_modbus_response(unit_id, ret, -1, timeout, t_prot_id=prot_id, ret_raw=True)

    def set_common_param(self, param_type, param_val):
        txdata = bytes([param_type])
        if param_type == 1:
            txdata += convert.fp32_to_bytes(param_val)
        else:
            txdata += convert.int32_to_bytes(param_val)
        return self.set_nu8(XCONF.UxbusReg.SET_COMMON_PARAM, txdata, 5)
    
    def get_common_param(self, param_type):
        txdata = bytes([param_type])
        ret = self.getset_nu8(XCONF.UxbusReg.GET_COMMON_PARAM, txdata, 1, -1)
        data = [0] * 2
        data[0] = ret[0]
        if ret[0] != XCONF.UxbusState.ERR_NOTTCP:
            if param_type == 1:
                data[1] = convert.bytes_to_fp32(ret[1:])
            else:
                data[1] = convert.bytes_to_u32(ret[1:])
        return data

    def get_common_info(self, param_type):
        txdata = bytes([param_type])
        ret = self.getset_nu8(XCONF.UxbusReg.GET_COMMON_INFO, txdata, 1, -1)
        data = [0] * 2
        data[0] = ret[0]
        if ret[0] != XCONF.UxbusState.ERR_NOTTCP:
            if param_type == 1 or param_type == 2:
                data[1] = ret[1]
            elif param_type == 50:
                data[1] = convert.bytes_to_fp32(ret[1:])
            elif param_type == 101:
                data[1] = ret[1]
                data.append(convert.bytes_to_fp32(ret[2:6]))
                data.append(convert.bytes_to_fp32(ret[6:10]))
            elif param_type in [102, 104]:
                data[1] = ret[1]
                data.append(convert.bytes_to_fp32(ret[2:]))
            elif param_type == 105:
                data[1] = convert.bytes_to_fp32(ret[1:])
                data.append(convert.bytes_to_fp32(ret[5:]))
            elif param_type in [103, 106]:
                data[1] = ret[1]
                data.extend(convert.bytes_to_fp32s(ret[2:], 7))
            else:
                data[0] = XCONF.UxbusState.ERR_PARAM
        return data
    
    def get_traj_speeding(self, rate):
        txdata = bytes([rate])
        ret = self.getset_nu8(XCONF.UxbusReg.GET_TRAJ_SPEEDING, txdata, 1, -1)
        data = [0] * 4
        data[0] = ret[0]
        if ret[0] != XCONF.UxbusState.ERR_NOTTCP:
            data[1] = convert.bytes_to_int32(ret[1:5])
            data[2] = convert.bytes_to_int32(ret[5:9])
            data[3] = round(math.degrees(convert.bytes_to_fp32(ret[9:])),3)
        return data
