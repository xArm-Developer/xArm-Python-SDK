#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import math
import time
import uuid
import warnings
from collections.abc import Iterable
from ..core.config.x_config import XCONF
from ..core.utils.log import logger
from .base import Base
from .gripper import Gripper
from .track import Track
from .base_board import BaseBoard
from .servo import Servo
from .record import Record
from .robotiq import RobotIQ
from .ft_sensor import FtSensor
from .modbus_tcp import ModbusTcp
from .parse import GcodeParser
from .code import APIState
from .decorator import xarm_is_connected, xarm_is_ready, xarm_wait_until_not_pause, xarm_wait_until_cmdnum_lt_max
from .utils import to_radian
try:
    # from ..tools.blockly_tool import BlocklyTool
    from ..tools.blockly import BlocklyTool
except:
    print('import BlocklyTool module failed')
    BlocklyTool = None

gcode_p = GcodeParser()


class XArm(Gripper, Servo, Record, RobotIQ, BaseBoard, Track, FtSensor, ModbusTcp):

    def __init__(self, port=None, is_radian=False, do_not_open=False, instance=None, **kwargs):
        super(XArm, self).__init__()
        kwargs['init'] = True
        self._api_instance = instance
        Base.__init__(self, port, is_radian, do_not_open, **kwargs)

    def _is_out_of_tcp_range(self, value, i):
        if not self._check_tcp_limit or self._stream_type != 'socket' or not self._enable_report:
            return False
        tcp_range = XCONF.Robot.TCP_LIMITS.get(self.axis).get(self.device_type, [])
        if 2 < i < len(tcp_range):  # only limit rotate
            limit = list(tcp_range[i])
            limit[0] += self._position_offset[i]
            limit[1] += self._position_offset[i]
            limit[0] += self._world_offset[i]
            limit[1] += self._world_offset[i]
            if limit[0] == limit[1]:
                return False
            if value < limit[0] - math.radians(0.1) or value > limit[1] + math.radians(0.1):
                self.log_api_info('API -> set_position -> out_of_tcp_range -> code={}, i={} value={}'.format(APIState.OUT_OF_RANGE, i, value), code=APIState.OUT_OF_RANGE)
                return True
        return False

    def _is_out_of_joint_range(self, angle, i):
        if not self._check_joint_limit or self._stream_type != 'socket' or not self._enable_report:
            return False
        joint_limit = XCONF.Robot.JOINT_LIMITS.get(self.axis).get(self.device_type, [])
        if i < len(joint_limit):
            angle_range = joint_limit[i]
            if angle < angle_range[0] - math.radians(0.1) or angle > angle_range[1] + math.radians(0.1):
                self.log_api_info('API -> set_servo_angle -> out_of_joint_range -> code={}, i={} value={}'.format(APIState.OUT_OF_RANGE, i, angle), code=APIState.OUT_OF_RANGE)
                return True
        return False
    
    def __wait_sync(self):
        while not self._is_sync or self._need_sync:
            if not self.connected:
                return APIState.NOT_CONNECTED
            elif self.has_error:
                return APIState.HAS_ERROR
            elif self.is_stop:
                return APIState.NOT_READY
            time.sleep(0.05)
        return 0

    def __update_tcp_motion_params(self, speed, acc, mvtime, pose=None):
        self._last_tcp_speed = speed
        self._last_tcp_acc = acc
        self._mvtime = mvtime
        if pose is not None:
            self._last_position = pose.copy()

    def __update_joint_motion_params(self, speed, acc, mvtime, angles=None):
        self._last_joint_speed = speed
        self._last_joint_acc = acc
        self._mvtime = mvtime
        if angles is not None:
            self._last_angles = angles.copy()

    def __get_tcp_motion_params(self, speed=None, mvacc=None, mvtime=None, **kwargs):
        speed = speed if speed is not None else kwargs.get('mvvelo', self._last_tcp_speed)
        # spd = self._last_tcp_speed if speed is None else min(max(float(speed), self._min_tcp_speed), self._max_tcp_speed)
        # acc = self._last_tcp_acc if mvacc is None else min(max(float(mvacc), self._min_tcp_acc), self._max_tcp_acc)
        spd = self._last_tcp_speed if speed is None else min(max(float(speed), self._min_tcp_speed), 1000)
        acc = self._last_tcp_acc if mvacc is None else min(max(float(mvacc), self._min_tcp_acc), 50000)
        mvt = self._mvtime if mvtime is None else mvtime
        return spd, acc, mvt

    def __get_joint_motion_params(self, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        speed = speed if speed is not None else kwargs.get('mvvelo', None)
        speed = self._last_joint_speed if speed is None else to_radian(speed, is_radian)
        mvacc = self._last_joint_acc if mvacc is None else to_radian(mvacc, is_radian)
        # spd = min(max(float(speed), self._min_joint_speed), self._max_joint_speed)
        # acc = min(max(float(mvacc), self._min_joint_acc), self._max_joint_acc)
        spd = min(max(float(speed), self._min_joint_speed), math.pi)
        acc = min(max(float(mvacc), self._min_joint_acc), 20)
        mvt = self._mvtime if mvtime is None else mvtime
        return spd, acc, mvt

    def _set_position_absolute(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None,
                               speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        tcp_pos = [
            self._last_position[0] if x is None else float(x),
            self._last_position[1] if y is None else float(y),
            self._last_position[2] if z is None else float(z),
            self._last_position[3] if roll is None else to_radian(roll, is_radian),
            self._last_position[4] if pitch is None else to_radian(pitch, is_radian),
            self._last_position[5] if yaw is None else to_radian(yaw, is_radian),
        ]
        motion_type = kwargs.get('motion_type', False)
        for i in range(3):
            if self._is_out_of_tcp_range(tcp_pos[i+3], i + 3):
                return APIState.OUT_OF_RANGE
        if kwargs.get('check', False):
            _, limit = self.is_tcp_limit(tcp_pos, True)
            if _ == 0 and limit is True:
                return APIState.TCP_LIMIT
        self._has_motion_cmd = True
        spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, mvtime, **kwargs)
        radius = radius if radius is not None else -1
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        if self.version_is_ge(1, 11, 100) or kwargs.get('debug', False):
            ret = self.arm_cmd.move_line_common(tcp_pos, spd, acc, mvt, radius, coord=0, is_axis_angle=False, only_check_type=only_check_type, motion_type=motion_type, feedback_key=feedback_key)
        else:
            if radius >= 0:
                ret = self.arm_cmd.move_lineb(tcp_pos, spd, acc, mvt, radius, only_check_type, motion_type=motion_type)
            else:
                ret = self.arm_cmd.move_line(tcp_pos, spd, acc, mvt, only_check_type, motion_type=motion_type)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0], is_move_cmd=True)
        self.log_api_info('API -> set_position -> code={}, pos={}, radius={}, velo={}, acc={}'.format(
            ret[0], tcp_pos, radius, spd, acc), code=ret[0])
        self._is_set_move = True
        self._only_check_result = 0
        if only_check_type > 0 and ret[0] == 0:
            self._only_check_result = ret[3]
            return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
        if only_check_type <= 0 and wait and ret[0] == 0:
            code = self.wait_move(timeout, trans_id=trans_id)
            self.__update_tcp_motion_params(spd, acc, mvt)
            self._sync()
            return code
        if only_check_type <= 0 and (ret[0] >= 0 or self.get_is_moving()):
            self.__update_tcp_motion_params(spd, acc, mvt, tcp_pos)
        return ret[0]

    def _set_position_relative(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None,
                               speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        motion_type = kwargs.get('motion_type', False)
        if self.version_is_ge(1, 8, 100):
            # use relative api
            tcp_pos = [
                0 if x is None else float(x),
                0 if y is None else float(y),
                0 if z is None else float(z),
                0 if roll is None else to_radian(roll, is_radian),
                0 if pitch is None else to_radian(pitch, is_radian),
                0 if yaw is None else to_radian(yaw, is_radian),
            ]
            self._has_motion_cmd = True
            spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, mvtime, **kwargs)
            radius = radius if radius is not None else -1
            feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
            ret = self.arm_cmd.move_relative(tcp_pos, spd, acc, mvt, radius, False, False, only_check_type, motion_type=motion_type, feedback_key=feedback_key)
            trans_id = self._get_feedback_transid(feedback_key, studio_wait)
            ret[0] = self._check_code(ret[0], is_move_cmd=True)
            self.log_api_info('API -> set_relative_position -> code={}, pos={}, radius={}, velo={}, acc={}'.format(
                ret[0], tcp_pos, radius, spd, acc), code=ret[0])
            self._is_set_move = True
            self._only_check_result = 0
            if only_check_type > 0 and ret[0] == 0:
                self._only_check_result = ret[3]
                return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
            if only_check_type <= 0 and wait and ret[0] == 0:
                code = self.wait_move(timeout, trans_id=trans_id)
                self.__update_tcp_motion_params(spd, acc, mvt)
                self._sync()
                return code
            if only_check_type <= 0 and (ret[0] >= 0 or self.get_is_moving()):
                self.__update_tcp_motion_params(spd, acc, mvt)
            return ret[0]
        else:
            # use absolute api
            tcp_pos = [
                self._last_position[0] if x is None else (self._last_position[0] + float(x)),
                self._last_position[1] if y is None else (self._last_position[1] + float(y)),
                self._last_position[2] if z is None else (self._last_position[2] + float(z)),
                self._last_position[3] if roll is None else (self._last_position[3] + to_radian(roll, is_radian)),
                self._last_position[4] if pitch is None else (self._last_position[4] + to_radian(pitch, is_radian)),
                self._last_position[5] if yaw is None else (self._last_position[5] + to_radian(yaw, is_radian)),
            ]
            return self._set_position_absolute(*tcp_pos, radius=radius, speed=speed, mvacc=mvacc, mvtime=mvtime,
                                               is_radian=True, wait=wait, timeout=timeout, **kwargs)

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None,
                     wait=False, timeout=None, **kwargs):
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        if only_check_type > 0 and wait:
            code = self.wait_move(timeout=timeout)
            if code != 0:
                return code
        code = self.__wait_sync()
        if code != 0:
            return code
        if relative:
            return self._set_position_relative(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, radius=radius,
                                               speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian,
                                               wait=wait, timeout=timeout, **kwargs)
        else:
            return self._set_position_absolute(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, radius=radius,
                                               speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian,
                                               wait=wait, timeout=timeout, **kwargs)

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_tool_position(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0,
                          speed=None, mvacc=None, mvtime=None, is_radian=None,
                          wait=False, timeout=None, radius=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        if only_check_type > 0 and wait:
            code = self.wait_move(timeout=timeout)
            if code != 0:
                return code
        tcp_pos = [
            x, y, z,
            to_radian(roll, is_radian),
            to_radian(pitch, is_radian),
            to_radian(yaw, is_radian)
        ]
        spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, mvtime, **kwargs)
        self._has_motion_cmd = True
        motion_type = kwargs.get('motion_type', False)
        radius = radius if radius is not None else -1
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        if self.version_is_ge(1, 11, 100) or kwargs.get('debug', False):
            ret = self.arm_cmd.move_line_common(tcp_pos, spd, acc, mvt, radius, coord=1, is_axis_angle=False, only_check_type=only_check_type, motion_type=motion_type, feedback_key=feedback_key)
        else:
            ret = self.arm_cmd.move_line_tool(tcp_pos, spd, acc, mvt, only_check_type, motion_type=motion_type)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0], is_move_cmd=True)
        self.log_api_info('API -> set_tool_position -> code={}, pos={}, velo={}, acc={}'.format(
            ret[0], tcp_pos, spd, acc), code=ret[0])
        self._is_set_move = True
        self._only_check_result = 0
        if only_check_type > 0 and ret[0] == 0:
            self._only_check_result = ret[3]
            return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
        if only_check_type <= 0 and wait and ret[0] == 0:
            code = self.wait_move(timeout, trans_id=trans_id)
            self.__update_tcp_motion_params(spd, acc, mvt)
            self._sync()
            return code
        if only_check_type <= 0 and (ret[0] >= 0 or self.get_is_moving()):
            self.__update_tcp_motion_params(spd, acc, mvt)
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_position_aa(self, mvpose, speed=None, mvacc=None, mvtime=None,
                        is_radian=None, is_tool_coord=False, relative=False,
                        wait=False, timeout=None, radius=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        if only_check_type > 0 and wait:
            code = self.wait_move(timeout=timeout)
            if code != 0:
                return code
        tcp_pos = [to_radian(mvpose[i], is_radian or i <= 2) for i in range(6)]
        spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, mvtime, **kwargs)
        mvcoord = kwargs.get('mvcoord', int(is_tool_coord))
        self._has_motion_cmd = True
        motion_type = kwargs.get('motion_type', False)
        radius = radius if radius is not None else -1
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        if self.version_is_ge(1, 11, 100) or kwargs.get('debug', False):
            if not is_tool_coord and relative:
                ret = self.arm_cmd.move_relative(tcp_pos, spd, acc, mvt, radius, False, True, only_check_type, motion_type=motion_type, feedback_key=feedback_key)
            else:
                ret = self.arm_cmd.move_line_common(tcp_pos, spd, acc, mvt, radius, coord=1 if is_tool_coord else 0, is_axis_angle=True, only_check_type=only_check_type, motion_type=motion_type, feedback_key=feedback_key)
        else:
            ret = self.arm_cmd.move_line_aa(tcp_pos, spd, acc, mvt, mvcoord, int(relative), only_check_type, motion_type=motion_type)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0], is_move_cmd=True)
        self.log_api_info('API -> set_position_aa -> code={}, pos={}, velo={}, acc={}'.format(
            ret[0], tcp_pos, spd, acc), code=ret[0])
        self._is_set_move = True
        self._only_check_result = 0
        if only_check_type > 0 and ret[0] == 0:
            self._only_check_result = ret[3]
            return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
        if only_check_type <= 0 and wait and ret[0] == 0:
            code = self.wait_move(timeout, trans_id=trans_id)
            self.__update_tcp_motion_params(spd, acc, mvt)
            self._sync()
            return code
        if only_check_type <= 0 and (ret[0] >= 0 or self.get_is_moving()):
            self.__update_tcp_motion_params(spd, acc, mvt)
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_is_ready(_type='set')
    def set_servo_cartesian_aa(self, mvpose, speed=None, mvacc=None, is_radian=None, is_tool_coord=False, relative=False, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert len(mvpose) >= 6
        tcp_pos = [to_radian(mvpose[i], is_radian or i <= 2) for i in range(6)]
        spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, self._mvtime, **kwargs)

        tool_coord = kwargs.get('tool_coord', int(is_tool_coord))

        self._has_motion_cmd = True
        ret = self.arm_cmd.move_servo_cart_aa(mvpose=tcp_pos, mvvelo=spd, mvacc=acc, tool_coord=tool_coord, relative=int(relative))
        ret[0] = self._check_code(ret[0], is_move_cmd=True, mode=1)
        self.log_api_info('API -> set_servo_cartesian_aa -> code={}, pose={}, velo={}, acc={}'.format(
            ret[0], tcp_pos, spd, acc
        ), code=ret[0])
        self._is_set_move = True
        return ret[0]

    def _set_servo_angle_absolute(self, angles, speed=None, mvacc=None, mvtime=None,
                                  is_radian=None, wait=False, timeout=None, radius=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        joints = self._last_angles.copy()
        for i in range(min(len(self._last_angles), len(angles))):
            if i >= self.axis or angles[i] is None:
                continue
            joints[i] = to_radian(angles[i], is_radian)
            if self._is_out_of_joint_range(joints[i], i):
                return APIState.OUT_OF_RANGE
        if kwargs.get('check', False):
            _, limit = self.is_joint_limit(joints, True)
            if _ == 0 and limit is True:
                return APIState.JOINT_LIMIT
        spd, acc, mvt = self.__get_joint_motion_params(speed, mvacc, mvtime, is_radian=is_radian, **kwargs)
        self._has_motion_cmd = True
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        if self.version_is_ge(1, 5, 20) and radius is not None and radius >= 0:
            ret = self.arm_cmd.move_jointb(joints, spd, acc, radius, only_check_type, feedback_key=feedback_key)
        else:
            ret = self.arm_cmd.move_joint(joints, spd, acc, mvt, only_check_type, feedback_key=feedback_key)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0], is_move_cmd=True)
        self.log_api_info('API -> set_servo_angle -> code={}, angles={}, velo={}, acc={}, radius={}'.format(
            ret[0], joints, spd, acc, radius
        ), code=ret[0])
        self._is_set_move = True
        self._only_check_result = 0
        if only_check_type > 0 and ret[0] == 0:
            self._only_check_result = ret[3]
            return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
        if only_check_type <= 0 and wait and ret[0] == 0:
            code = self.wait_move(timeout, trans_id=trans_id)
            self.__update_joint_motion_params(spd, acc, mvt)
            self._sync()
            return code
        if only_check_type <= 0 and (ret[0] >= 0 or self.get_is_moving()):
            self.__update_joint_motion_params(spd, acc, mvt, joints)
        return ret[0]

    def _set_servo_angle_relative(self, angles, speed=None, mvacc=None, mvtime=None,
                                  is_radian=None, wait=False, timeout=None, radius=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        if self.version_is_ge(1, 8, 100):
            # use relative api
            joints = [0] * 7
            for i in range(min(7, len(angles))):
                if i >= self.axis or angles[i] is None:
                    continue
                joints[i] = to_radian(angles[i], is_radian)
            self._has_motion_cmd = True
            spd, acc, mvt = self.__get_joint_motion_params(speed, mvacc, mvtime, is_radian=is_radian, **kwargs)
            radius = radius if radius is not None else -1
            feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
            ret = self.arm_cmd.move_relative(joints, spd, acc, mvt, radius, True, False, only_check_type, feedback_key=feedback_key)
            trans_id = self._get_feedback_transid(feedback_key, studio_wait)
            ret[0] = self._check_code(ret[0], is_move_cmd=True)
            self.log_api_info('API -> set_relative_servo_angle -> code={}, angles={}, velo={}, acc={}, radius={}'.format(
                ret[0], joints, spd, acc, radius
            ), code=ret[0])
            self._is_set_move = True
            self._only_check_result = 0
            if only_check_type > 0 and ret[0] == 0:
                self._only_check_result = ret[3]
                return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
            if only_check_type <= 0 and wait and ret[0] == 0:
                code = self.wait_move(timeout, trans_id=trans_id)
                self.__update_joint_motion_params(spd, acc, mvt)
                self._sync()
                return code
            if only_check_type <= 0 and (ret[0] >= 0 or self.get_is_moving()):
                self.__update_joint_motion_params(spd, acc, mvt)
            return ret[0]
        else:
            # use absolute api
            joints = self._last_angles.copy()
            for i in range(min(len(self._last_angles), len(angles))):
                if i >= self.axis or angles[i] is None:
                    continue
                joints[i] = to_radian(angles[i], is_radian)
                if self._is_out_of_joint_range(joints[i], i):
                    return APIState.OUT_OF_RANGE
            return self._set_servo_angle_absolute(joints, speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=True,
                                                  wait=wait, timeout=timeout, radius=radius, **kwargs)

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=None, wait=False, timeout=None, radius=None, **kwargs):
        assert ((servo_id is None or servo_id == 8) and isinstance(angle, Iterable)) \
            or (1 <= servo_id <= 7 and angle is not None and not isinstance(angle, Iterable)), \
            'param servo_id or angle error'
        if servo_id is not None and servo_id != 8:
            if servo_id > self.axis or servo_id <= 0:
                return APIState.SERVO_NOT_EXIST
            angles = [None] * 7
            angles[servo_id - 1] = angle
        else:
            angles = angle
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        if only_check_type > 0 and wait:
            code = self.wait_move(timeout=timeout)
            if code != 0:
                return code
        code = self.__wait_sync()
        if code != 0:
            return code
        if relative:
            return self._set_servo_angle_relative(angles, speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian,
                                                  wait=wait, timeout=timeout, radius=radius, **kwargs)
        else:
            return self._set_servo_angle_absolute(angles, speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian,
                                                  wait=wait, timeout=timeout, radius=radius, **kwargs)

    @xarm_is_ready(_type='set')
    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        # if not self._check_mode_is_correct(1):
        #     return APIState.MODE_IS_NOT_CORRECT
        is_radian = self._default_is_radian if is_radian is None else is_radian
        angs = [to_radian(angle, is_radian) for angle in angles]
        for i in range(self.axis):
            if self._is_out_of_joint_range(angs[i], i):
                return APIState.OUT_OF_RANGE
        while len(angs) < 7:
            angs.append(0)
        spd, acc, mvt = self.__get_joint_motion_params(speed, mvacc, mvtime, is_radian=is_radian, **kwargs)
        self._has_motion_cmd = True
        ret = self.arm_cmd.move_servoj(angs, spd, acc, mvt)
        ret[0] = self._check_code(ret[0], is_move_cmd=True, mode=1)
        self.log_api_info('API -> set_servo_angle_j -> code={}, angles={}, velo={}, acc={}'.format(
            ret[0], angs, spd, acc
        ), code=ret[0])
        self._is_set_move = True
        return ret[0]

    @xarm_is_ready(_type='set')
    def set_servo_cartesian(self, mvpose, speed=None, mvacc=None, mvtime=None, is_radian=None, is_tool_coord=False, **kwargs):
        # if not self._check_mode_is_correct(1):
        #     return APIState.MODE_IS_NOT_CORRECT
        assert len(mvpose) >= 6
        is_radian = self._default_is_radian if is_radian is None else is_radian
        tcp_pos = [to_radian(mvpose[i], is_radian or i <= 2) for i in range(6)]
        spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, mvtime, **kwargs)
        self._has_motion_cmd = True
        ret = self.arm_cmd.move_servo_cartesian(tcp_pos, spd, acc, int(is_tool_coord))
        ret[0] = self._check_code(ret[0], is_move_cmd=True, mode=1)
        self.log_api_info('API -> set_servo_cartisian -> code={}, pose={}, velo={}, acc={}, is_tool_coord={}'.format(
            ret[0], tcp_pos, spd, acc, is_tool_coord
        ), code=ret[0])
        self._is_set_move = True
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def move_circle(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None,
                    wait=False, timeout=None, is_tool_coord=False, is_axis_angle=False, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        if only_check_type > 0 and wait:
            code = self.wait_move(timeout=timeout)
            if code != 0:
                return code
        pose_1 = []
        pose_2 = []
        for i in range(6):
            pose_1.append(to_radian(pose1[i], is_radian or i <= 2))
            pose_2.append(to_radian(pose2[i], is_radian or i <= 2))
        spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, mvtime, **kwargs)
        self._has_motion_cmd = True
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        if self.version_is_ge(1, 11, 100) or kwargs.get('debug', False):
            ret = self.arm_cmd.move_circle_common(pose_1, pose_2, spd, acc, mvt, percent, coord=1 if is_tool_coord else 0, is_axis_angle=is_axis_angle, only_check_type=only_check_type, feedback_key=feedback_key)
        else:
            ret = self.arm_cmd.move_circle(pose_1, pose_2, spd, acc, mvt, percent, only_check_type)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0], is_move_cmd=True)
        self.log_api_info('API -> move_circle -> code={}, pos1={}, pos2={}, percent={}%, velo={}, acc={}'.format(
            ret[0], pose_1, pose_2, percent, spd, acc), code=ret[0])
        self._is_set_move = True
        self._only_check_result = 0
        if only_check_type > 0 and ret[0] == 0:
            self._only_check_result = ret[3]
            return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
        if only_check_type <= 0 and wait and ret[0] == 0:
            code = self.wait_move(timeout, trans_id=trans_id)
            self.__update_tcp_motion_params(spd, acc, mvt)
            self._sync()
            return code
        if only_check_type <= 0 and (ret[0] >= 0 or self.get_is_moving()):
            self.__update_tcp_motion_params(spd, acc, mvt)
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        only_check_type = kwargs.get('only_check_type', self._only_check_type)
        if only_check_type > 0 and wait:
            code = self.wait_move(timeout=timeout)
            if code != 0:
                return code
        spd, acc, mvt = self.__get_joint_motion_params(speed, mvacc, mvtime, is_radian=is_radian, **kwargs)
        self._has_motion_cmd = True
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        ret = self.arm_cmd.move_gohome(spd, acc, mvt, only_check_type, feedback_key=feedback_key)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0], is_move_cmd=True)
        self.log_api_info('API -> move_gohome -> code={}, velo={}, acc={}'.format(
            ret[0], spd, acc
        ), code=ret[0])
        self._is_set_move = True
        self._only_check_result = 0
        if only_check_type > 0 and ret[0] == 0:
            self._only_check_result = ret[3]
            return APIState.HAS_ERROR if ret[3] != 0 else ret[0]
        if only_check_type <= 0 and wait and ret[0] == 0:
            code = self.wait_move(timeout, trans_id=trans_id)
            self._sync()
            return code
        return ret[0]

    @xarm_is_ready(_type='set')
    def move_arc_lines(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0,
                       automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):
        assert len(paths) > 0, 'parameter paths error'
        is_radian = self._default_is_radian if is_radian is None else is_radian
        spd, acc, mvt = self.__get_tcp_motion_params(speed, mvacc, mvtime)
        logger.info('move_arc_lines--begin')
        if automatic_calibration:
            _ = self.set_position(*paths[0], is_radian=is_radian, speed=spd, mvacc=acc, mvtime=mvt, wait=True)
            if _ < 0:
                logger.error('quit, api failed, code={}'.format(_))
                return
            _, angles = self.get_servo_angle(is_radian=True)
        if first_pause_time > 0:
            self.set_pause_time(first_pause_time)
        last_used_joint_speed = self._last_joint_speed

        def _move():
            if automatic_calibration:
                ret = self.set_servo_angle(angle=angles, is_radian=True, speed=0.8726646259971648, wait=False)
                if ret < 0:
                    return -1
                self._last_joint_speed = last_used_joint_speed
            for path in paths:
                if len(path) > 6 and path[6] >= 0:
                    radius = path[6]
                else:
                    radius = 0
                if self.has_error or self.is_stop:
                    return -2
                ret = self.set_position(*path[:6], radius=radius, is_radian=is_radian, wait=False, speed=spd, mvacc=acc, mvtime=mvt)
                if ret < 0:
                    return -1
            return 0
        count = 1
        api_failed = False

        try:
            if times == 0:
                while not self.has_error and not self.is_stop:
                    _ = _move()
                    if _ == -1:
                        api_failed = True
                        break
                    elif _ == -2:
                        break
                    count += 1
                    if repeat_pause_time > 0:
                        self.set_pause_time(repeat_pause_time)
                if api_failed:
                    logger.error('quit, api error')
                elif self._error_code != 0:
                    logger.error('quit, controller error')
                elif self.is_stop:
                    logger.error('quit, emergency_stop')
            else:
                for i in range(times):
                    if self.has_error or self.is_stop:
                        break
                    _ = _move()
                    if _ == -1:
                        api_failed = True
                        break
                    elif _ == -2:
                        break
                    count += 1
                    if repeat_pause_time > 0:
                        self.set_pause_time(repeat_pause_time)
                if api_failed:
                    logger.error('quit, api error')
                elif self._error_code != 0:
                    logger.error('quit, controller error')
                elif self.is_stop:
                    logger.error('quit, emergency_stop')
        except:
            pass
        logger.info('move_arc_lines--end')
        if wait:
            self.wait_move()
            self._sync()

    @xarm_is_connected(_type='set')
    def set_servo_attach(self, servo_id=None):
        # assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        # ret = self.arm_cmd.set_brake(servo_id, 0)
        logger.info('set_servo_attach--begin')
        ret = self.motion_enable(servo_id=servo_id, enable=True)
        self.set_state(0)
        self._sync()
        logger.info('set_servo_attach--end')
        return ret

    @xarm_is_connected(_type='set')
    def set_servo_detach(self, servo_id=None):
        """
        :param servo_id: 1-7, 8
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8, 'The value of parameter servo_id can only be 1-8.'
        ret = self.arm_cmd.set_brake(servo_id, 1)
        self.log_api_info('API -> set_servo_detach -> code={}'.format(ret[0]), code=ret[0])
        self._sync()
        return ret[0]

    @xarm_is_connected(_type='set')
    def system_control(self, value=1):
        ret = self.arm_cmd.system_control(value)
        self.log_api_info('API -> system_control({}) -> code={}'.format(value, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_reduced_mode(self, on_off):
        ret = self.arm_cmd.set_reduced_mode(int(on_off))
        self.log_api_info('API -> set_reduced_mode -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_reduced_max_tcp_speed(self, speed):
        ret = self.arm_cmd.set_reduced_linespeed(speed)
        self.log_api_info('API -> set_reduced_linespeed -> code={}, speed={}'.format(ret[0], speed), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_reduced_max_joint_speed(self, speed, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        speed = to_radian(speed, is_radian)
        ret = self.arm_cmd.set_reduced_jointspeed(speed)
        self.log_api_info('API -> set_reduced_linespeed -> code={}, speed={}'.format(ret[0], speed), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_reduced_mode(self):
        ret = self.arm_cmd.get_reduced_mode()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_reduced_states(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_reduced_states(79 if self.version_is_ge(1, 2, 11) else 21)
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            if not is_radian:
                ret[4] = round(math.degrees(ret[4]), 1)
                if self.version_is_ge(1, 2, 11):
                    # ret[5] = list(map(math.degrees, ret[5]))
                    ret[5] = list(map(lambda x: round(math.degrees(x), 2), ret[5]))
        return ret[0], ret[1:]

    @xarm_is_connected(_type='set')
    def set_reduced_tcp_boundary(self, boundary):
        assert len(boundary) >= 6
        boundary = list(map(int, boundary))
        limits = [0] * 6
        limits[0:2] = boundary[0:2] if boundary[0] >= boundary[1] else boundary[0:2][::-1]
        limits[2:4] = boundary[2:4] if boundary[2] >= boundary[3] else boundary[2:4][::-1]
        limits[4:6] = boundary[4:6] if boundary[4] >= boundary[5] else boundary[4:6][::-1]
        ret = self.arm_cmd.set_xyz_limits(limits)
        self.log_api_info('API -> set_reduced_tcp_boundary -> code={}, boundary={}'.format(ret[0], limits), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_reduced_joint_range(self, joint_range, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert len(joint_range) >= self.axis * 2
        joint_range = list(map(float, joint_range))
        limits = [0] * 14
        for i in range(7):
            if i < self.axis:
                limits[i*2:i*2+2] = joint_range[i*2:i*2+2] if joint_range[i*2] <= joint_range[i*2+1] else joint_range[i*2:i*2+2][::-1]
        if not is_radian:
            limits = list(map(math.radians, limits))
            # limits = list(map(lambda x: round(math.radians(x), 3), limits))

        for i in range(self.axis):
            joint_limit = XCONF.Robot.JOINT_LIMITS.get(self.axis).get(self.device_type, [])
            if i < len(joint_limit):
                angle_range = joint_limit[i]
                # angle_range = list(map(lambda x: round(x, 3), joint_limit[i]))
                if limits[i * 2] < angle_range[0]:
                    limits[i * 2] = angle_range[0]
                if limits[i * 2 + 1] > angle_range[1]:
                    limits[i * 2 + 1] = angle_range[1]
                if limits[i * 2] >= angle_range[1]:
                    return APIState.OUT_OF_RANGE
                if limits[i * 2 + 1] <= angle_range[0]:
                    return APIState.OUT_OF_RANGE
        ret = self.arm_cmd.set_reduced_jrange(limits)
        self.log_api_info('API -> set_reduced_joint_range -> code={}, boundary={}'.format(ret[0], limits), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_fense_mode(self, on_off):
        ret = self.arm_cmd.set_fense_on(int(on_off))
        self.log_api_info('API -> set_fense_mode -> code={}, on={}'.format(ret[0], on_off), code=ret[0])
        return ret

    @xarm_is_connected(_type='set')
    def set_collision_rebound(self, on_off):
        ret = self.arm_cmd.set_collis_reb(int(on_off))
        self.log_api_info('API -> set_collision_rebound -> code={}, on={}'.format(ret[0], on_off), code=ret[0])
        return ret

    @xarm_is_connected(_type='set')
    def set_timer(self, secs_later, tid, fun_code, param1=0, param2=0):
        ret = self.arm_cmd.set_timer(secs_later, tid, fun_code, param1, param2)
        return ret[0]

    @xarm_is_connected(_type='set')
    def cancel_timer(self, tid):
        ret = self.arm_cmd.cancel_timer(tid)
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_is_connected(_type='set')
    def set_world_offset(self, offset, is_radian=None, wait=True):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert isinstance(offset, Iterable) and len(offset) >= 6
        world_offset = [0] * 6
        for i in range(min(len(offset), 6)):
            world_offset[i] = to_radian(offset[i], is_radian or i <= 2)
        if wait:
            if self._support_feedback:
                self.wait_all_task_finish()
            else:
                self.wait_move()
        ret = self.arm_cmd.set_world_offset(world_offset)
        self.log_api_info('API -> set_world_offset -> code={}, offset={}'.format(ret[0], world_offset), code=ret[0])
        return ret[0]

    def reset(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):
        logger.info('reset--begin')
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if not self._enable_report or self._stream_type != 'socket':
            self.get_err_warn_code()
            self.get_state()
        if self._warn_code != 0:
            self.clean_warn()
        if self._error_code != 0:
            self.clean_error()
            self.motion_enable(enable=True, servo_id=8)
            self.set_state(0)
        if not self._is_ready:
            self.motion_enable(enable=True, servo_id=8)
            self.set_state(state=0)
        self.move_gohome(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)
        logger.info('reset--end')

    # # This interface is no longer supported
    # @xarm_is_ready(_type='set')
    # def set_joints_torque(self, joints_torque):
    #     ret = self.arm_cmd.set_servot(joints_torque)
    #     self.log_api_info('API -> set_joints_torque -> code={}, joints_torque={}'.format(ret[0], joints_torque), code=ret[0])
    #     return ret[0]

    @xarm_is_connected(_type='get')
    def get_joints_torque(self, servo_id=None):
        ret = self.arm_cmd.get_joint_tau()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and len(ret) > 7:
            self._joints_torque = [float('{:.6f}'.format(ret[i])) for i in range(1, 8)]
        if servo_id is None or servo_id == 8 or len(self._joints_torque) < servo_id:
            return ret[0], list(self._joints_torque)
        else:
            return ret[0], self._joints_torque[servo_id - 1]

    @xarm_is_connected(_type='get')
    def get_safe_level(self):
        ret = self.arm_cmd.get_safe_level()
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_safe_level(self, level=4):
        ret = self.arm_cmd.set_safe_level(level)
        self.log_api_info('API -> set_safe_level -> code={}, level={}'.format(ret[0], level), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_pause_time(self, sltime, wait=False):
        assert isinstance(sltime, (int, float))
        ret = self.arm_cmd.sleep_instruction(sltime)
        if wait:
            time.sleep(sltime)
        else:
            if time.monotonic() >= self._sleep_finish_time:
                self._sleep_finish_time = time.monotonic() + sltime
            else:
                self._sleep_finish_time += sltime
        self.log_api_info('API -> set_pause_time -> code={}, sltime={}'.format(ret[0], sltime), code=ret[0])
        return ret[0]

    def set_sleep_time(self, sltime, wait=False):
        return self.set_pause_time(sltime, wait)

    @xarm_wait_until_not_pause
    @xarm_is_connected(_type='set')
    def set_tcp_offset(self, offset, is_radian=None, wait=True, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert isinstance(offset, Iterable) and len(offset) >= 6
        tcp_offset = [0] * 6
        for i in range(min(len(offset), 6)):
            tcp_offset[i] = to_radian(offset[i], is_radian or i <= 2)
        if wait:
            if self._support_feedback:
                self.wait_all_task_finish()
            else:
                self.wait_move()
        ret = self.arm_cmd.set_tcp_offset(tcp_offset)
        self.log_api_info('API -> set_tcp_offset -> code={}, offset={}'.format(ret[0], tcp_offset), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_tcp_jerk(self, jerk):
        ret = self.arm_cmd.set_tcp_jerk(jerk)
        self.log_api_info('API -> set_tcp_jerk -> code={}, jerk={}'.format(ret[0], jerk), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_tcp_maxacc(self, acc):
        ret = self.arm_cmd.set_tcp_maxacc(acc)
        self.log_api_info('API -> set_tcp_maxacc -> code={}, maxacc={}'.format(ret[0], acc), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_joint_jerk(self, jerk, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        jerk = to_radian(jerk, is_radian)
        ret = self.arm_cmd.set_joint_jerk(jerk)
        self.log_api_info('API -> set_joint_jerk -> code={}, jerk={}'.format(ret[0], jerk), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_joint_maxacc(self, maxacc, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        maxacc = to_radian(maxacc, is_radian)
        ret = self.arm_cmd.set_joint_maxacc(maxacc)
        self.log_api_info('API -> set_joint_maxacc -> code={}, maxacc={}'.format(ret[0], maxacc), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_is_connected(_type='set')
    def set_collision_sensitivity(self, value, wait=True):
        assert isinstance(value, int) and 0 <= value <= 5
        if self._support_feedback:
            self.wait_all_task_finish()
        else:
            self.wait_move()
        ret = self.arm_cmd.set_collis_sens(value)
        self.set_state(0)
        self.log_api_info('API -> set_collision_sensitivity -> code={}, sensitivity={}'.format(ret[0], value), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_is_connected(_type='set')
    def set_teach_sensitivity(self, value, wait=True):
        assert isinstance(value, int) and 1 <= value <= 5
        if wait:
            if self._support_feedback:
                self.wait_all_task_finish()
            else:
                self.wait_move()
        ret = self.arm_cmd.set_teach_sens(value)
        self.log_api_info('API -> set_teach_sensitivity -> code={}, sensitivity={}'.format(ret[0], value), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_is_connected(_type='set')
    def set_gravity_direction(self, direction, wait=True):
        if wait:
            if self._support_feedback:
                self.wait_all_task_finish()
            else:
                self.wait_move()
        ret = self.arm_cmd.set_gravity_dir(direction[:3])
        self.log_api_info('API -> set_gravity_direction -> code={}, direction={}'.format(ret[0], direction), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_is_connected(_type='set')
    def set_mount_direction(self, base_tilt_deg, rotation_deg, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        t1 = base_tilt_deg
        t2 = rotation_deg

        if not is_radian:
            t1 = math.radians(t1)
            t2 = math.radians(t2)

        # original G vect mounted on flat surface
        G_normal = [0, 0, -1]

        # rotation matrix introduced by 2 mounting angles
        R2 = [math.cos(-t2), -math.sin(-t2), 0, math.sin(-t2), math.cos(-t2), 0, 0, 0, 1]
        R1 = [math.cos(-t1), 0, math.sin(-t1), 0, 1, 0, -math.sin(-t1), 0, math.cos(-t1)]

        Rot = [0] * 9
        g_new = [0] * 3

        # Mat(Rot) = Mat(R2)*Mat(R1)
        # vect(g_new) = Mat(Rot)*vect(G_normal)
        for i in range(3):
            for j in range(3):
                Rot[i * 3 + j] += (
                R2[i * 3 + 0] * R1[0 * 3 + j] + R2[i * 3 + 1] * R1[1 * 3 + j] + R2[i * 3 + 2] * R1[2 * 3 + j])

            g_new[i] = Rot[i * 3 + 0] * G_normal[0] + Rot[i * 3 + 1] * G_normal[1] + Rot[i * 3 + 2] * G_normal[2]

        ret = self.arm_cmd.set_gravity_dir(g_new)
        self.log_api_info('API -> set_mount_direction -> code={}, tilt={}, rotation={}, direction={}'.format(ret[0], base_tilt_deg, rotation_deg, g_new), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_conf(self):
        ret = self.arm_cmd.clean_conf()
        self.log_api_info('API -> clean_conf -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def save_conf(self):
        ret = self.arm_cmd.save_conf()
        self.log_api_info('API -> save_conf -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        assert len(pose) >= 6
        tcp_pose = [to_radian(pose[i], input_is_radian or i <= 2) for i in range(6)]
        ret = self.arm_cmd.get_ik(tcp_pose)
        angles = []
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            # angles = [ret[i][0] for i in range(1, 8)]
            angles = [ret[i] for i in range(1, 8)]
            if not return_is_radian:
                angles = [math.degrees(angle) for angle in angles]
        return ret[0], angles

    @xarm_is_connected(_type='get')
    def get_forward_kinematics(self, angles, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        # assert len(angles) >= 7
        joints = [0] * 7
        for i in range(min(len(angles), 7)):
            joints[i] = to_radian(angles[i], input_is_radian)

        ret = self.arm_cmd.get_fk(joints)
        pose = []
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            # pose = [ret[i][0] for i in range(1, 7)]
            pose = [ret[i] for i in range(1, 7)]
            if not return_is_radian:
                pose = [pose[i] if i < 3 else math.degrees(pose[i]) for i in range(len(pose))]
        return ret[0], pose

    @xarm_is_connected(_type='get')
    def is_tcp_limit(self, pose, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert len(pose) >= 6
        tcp_pose = [to_radian(pose[i], is_radian or i <= 2, self._last_position[i]) for i in range(6)]
        ret = self.arm_cmd.is_tcp_limit(tcp_pose)
        self.log_api_info('API -> is_tcp_limit -> code={}, limit={}'.format(ret[0], ret[1]), code=ret[0])
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            return ret[0], bool(ret[1])
        else:
            return ret[0], None

    @xarm_is_connected(_type='get')
    def is_joint_limit(self, joint, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        # assert len(joint) >= 7
        joints = [0] * 7
        for i in range(min(len(joint), 7)):
            joints[i] = to_radian(joint[i], is_radian, self._last_angles[i])

        ret = self.arm_cmd.is_joint_limit(joints)
        self.log_api_info('API -> is_joint_limit -> code={}, limit={}'.format(ret[0], ret[1]), code=ret[0])
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            return ret[0], bool(ret[1])
        else:
            return ret[0], None

    def emergency_stop(self):
        logger.info('emergency_stop--begin')
        self.set_state(4)
        expired = time.monotonic() + 3
        while self.state not in [4] and time.monotonic() < expired:
            self.set_state(4)
            time.sleep(0.1)
        self._sleep_finish_time = 0
        self._sync()
        logger.info('emergency_stop--end')

    def send_cmd_async(self, command, timeout=None):
        pass

    def send_cmd_sync(self, command=None):
        if command is None:
            return 0
        command = command.upper()
        return self._handle_gcode(command)

    def _handle_gcode(self, command):
        def __handle_gcode_g(num):
            if num == 1:  # G1 move_line, ex: G1 X{} Y{} Z{} A{roll} B{pitch} C{yaw} F{speed} Q{acc} T{}
                mvvelo = gcode_p.get_mvvelo(command)
                mvacc = gcode_p.get_mvacc(command)
                mvtime = gcode_p.get_mvtime(command)
                mvpose = gcode_p.get_poses(command)
                ret = self.set_position(*mvpose, radius=-1, speed=mvvelo, mvacc=mvacc, mvtime=mvtime)
            elif num == 2:  # G2 move_circle, ex: G2 X{} Y{} Z{} A{} B{} C{} I{} J{} K{} L{} M{} N{} F{speed} Q{acc} T{}
                mvvelo = gcode_p.get_mvvelo(command)
                mvacc = gcode_p.get_mvacc(command)
                mvtime = gcode_p.get_mvtime(command)
                pos1 = gcode_p.get_poses(command, default=0)
                pos2 = gcode_p.get_joints(command, default=0)[:6]
                percent = gcode_p.get_mvradius(command, default=0)
                ret = self.move_circle(pos1, pos2, percent=percent, speed=mvvelo, mvacc=mvacc, mvtime=mvtime)
            elif num == 4:  # G4 set_pause_time, ex: G4 T{}
                sltime = gcode_p.get_mvtime(command, default=0)
                ret = self.set_pause_time(sltime)
            elif num == 7:  # G7 move_joint, ex: G7 I{} J{} K{} L{} M{} N{} O{} F{} Q{} T{}
                mvvelo = gcode_p.get_mvvelo(command)
                mvacc = gcode_p.get_mvacc(command)
                mvtime = gcode_p.get_mvtime(command)
                mvjoint = gcode_p.get_joints(command)
                ret = self.set_servo_angle(angle=mvjoint, speed=mvvelo, mvacc=mvacc, mvtime=mvtime)
            elif num == 8:  # G8 move_gohome, ex: G8 F{} Q{} T{}
                mvvelo = gcode_p.get_mvvelo(command)
                mvacc = gcode_p.get_mvacc(command)
                mvtime = gcode_p.get_mvtime(command)
                ret = self.move_gohome(speed=mvvelo, mvacc=mvacc, mvtime=mvtime)
            elif num == 9:  # G9 move_arc_line, ex: G9 X{} Y{} Z{} A{roll} B{pitch} C{yaw} R{radius} F{speed} Q{acc} T{}
                mvvelo = gcode_p.get_mvvelo(command)
                mvacc = gcode_p.get_mvacc(command)
                mvtime = gcode_p.get_mvtime(command)
                mvpose = gcode_p.get_poses(command)
                mvradii = gcode_p.get_mvradius(command, default=0)
                ret = self.set_position(*mvpose, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, radius=mvradii)
            elif num == 11:  # G11 set_servo_angle_j, ex: G11 I{} J{} K{} L{} M{} N{} O{} F{} Q{} T{}
                mvvelo = gcode_p.get_mvvelo(command)
                mvacc = gcode_p.get_mvacc(command)
                mvtime = gcode_p.get_mvtime(command)
                mvjoint = gcode_p.get_joints(command, default=0)
                ret = self.set_servo_angle_j(mvjoint, speed=mvvelo, mvacc=mvacc, mvtime=mvtime)
            elif num == 12:  # G12 sleep, ex: G12 T{}
                mvtime = gcode_p.get_mvtime(command, default=0)
                time.sleep(mvtime)
                ret = 0
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
            return ret

        def __handle_gcode_h(num):
            if num == 1:  # H1 get_version, ex: H1
                ret = self.get_version()
            elif num == 10:  # H10 system_control, ex: H10 V{}
                value = gcode_p.get_int_value(command, default=0)
                ret = self.system_control(value)
            elif num == 11:  # H11 motion_enable, ex: H11 I{id} V{enable}
                value = gcode_p.get_int_value(command)
                servo_id = gcode_p.get_id_num(command, default=0)
                ret = self.motion_enable(enable=value, servo_id=servo_id)
            elif num == 12:  # H12 set_state, ex: H12 V{state}
                value = gcode_p.get_int_value(command, default=0)
                ret = self.set_state(value)
            elif num == 13:  # H13 get_state, ex: H13
                ret = self.get_state()
            elif num == 14:  # H14 get_cmd_num, ex: H14
                ret = self.get_cmdnum()
            elif num == 15:  # H15 get_error_warn_code, ex: H15
                ret = self.get_err_warn_code()
            elif num == 16:  # H16 clean_error, ex: H16
                ret = self.clean_error()
            elif num == 17:  # H17 clean_warn, ex: H17
                ret = self.clean_warn()
            elif num == 18:  # H18 set_brake, ex: H18 I{id} V{open}
                value = gcode_p.get_int_value(command)
                servo_id = gcode_p.get_id_num(command, default=0)
                ret = self.arm_cmd.set_brake(servo_id, value)[0]
            elif num == 19:  # H19 set_mode, ex: H19 V{mode}
                value = gcode_p.get_int_value(command, default=0)
                ret = self.set_mode(value)
            elif num == 31:  # H31 set_tcp_jerk, ex: H31 V{jerk}
                value = gcode_p.get_float_value(command, default=-1)
                ret = self.set_tcp_jerk(value)
            elif num == 32:  # H32 set_tcp_maxacc, ex: H32 V{maxacc}
                value = gcode_p.get_float_value(command, default=-1)
                ret = self.set_tcp_maxacc(value)
            elif num == 33:  # H33 set_joint_jerk, ex: H33 V{jerk}
                value = gcode_p.get_float_value(command, default=-1)
                ret = self.set_joint_jerk(value)
            elif num == 34:  # H34 set_joint_maxacc, ex: H34 V{maxacc}
                value = gcode_p.get_float_value(command, default=-1)
                ret = self.set_joint_maxacc(value)
            elif num == 35:  # H35 set_tcp_offset, ex: H35 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw}
                pose = gcode_p.get_poses(command)
                ret = self.set_tcp_offset(pose)
            elif num == 36:  # H36 set_tcp_load, ex: H36 I{weight} J{center_x} K{center_y} L{center_z}
                values = gcode_p.get_joints(command, default=0)
                ret = self.set_tcp_load(values[0], values[1:4])
            elif num == 37:  # H37 set_collision_sensitivity, ex: H37 V{sensitivity}
                value = gcode_p.get_int_value(command, default=0)
                ret = self.set_collision_sensitivity(value)
            elif num == 38:  # H38 set_teach_sensitivity, ex: H38 V{sensitivity}
                value = gcode_p.get_int_value(command, default=0)
                ret = self.set_teach_sensitivity(value)
            elif num == 39:  # H39 clean_conf, ex: H39
                ret = self.clean_conf()
            elif num == 40:  # H40 save_conf, ex: H40
                ret = self.save_conf()
            elif num == 41:  # H41 get_position, ex: H41
                ret = self.get_position()
            elif num == 42:  # H42 get_servo_angle, ex: H42
                ret = self.get_servo_angle()
            elif num == 43:  # H43 get_ik, ex: H43 X{} Y{} Z{} A{roll} B{pitch} C{yaw}
                pose = gcode_p.get_poses(command, default=0)
                ret = self.get_inverse_kinematics(pose, input_is_radian=False, return_is_radian=False)
            elif num == 44:  # H44 get_fk, ex: H44 I{} J{} K{} L{} M{} N{} O{}
                joint = gcode_p.get_joints(command, default=0)
                ret = self.get_forward_kinematics(joint, input_is_radian=False, return_is_radian=False)
            elif num == 45:  # H45 is_joint_limit, ex: H45 I{} J{} K{} L{} M{} N{} O{}
                joint = gcode_p.get_joints(command)
                ret = self.is_joint_limit(joint, is_radian=False)
            elif num == 46:  # H46 is_tcp_limit, ex: H46 X{} Y{} Z{} A{roll} B{pitch} C{yaw}
                pose = gcode_p.get_poses(command)
                ret = self.is_tcp_limit(pose, is_radian=False)
            elif num == 51:  # H51 set_gravity_direction, ex: H51 X{} Y{} Z{} A{roll} B{pitch} C{yaw}
                pose = gcode_p.get_poses(command, default=0)
                ret = self.set_gravity_direction(pose)
            elif num == 101:  # H101 set_servo_addr_16, ex: H101 I{id} D{addr} V{value}
                value = gcode_p.get_int_value(command)
                servo_id = gcode_p.get_id_num(command, default=0)
                addr = gcode_p.get_addr(command)
                ret = self.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)
            elif num == 102:  # H102 get_servo_addr_16, ex: H102 I{id} D{addr}
                servo_id = gcode_p.get_id_num(command, default=0)
                addr = gcode_p.get_addr(command)
                ret = self.get_servo_addr_16(servo_id=servo_id, addr=addr)
            elif num == 103:  # H103 set_servo_addr_32, ex: H103 I{id} D{addr} V{value}
                servo_id = gcode_p.get_id_num(command, default=0)
                addr = gcode_p.get_addr(command)
                value = gcode_p.get_int_value(command)
                ret = self.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)
            elif num == 104:  # H104 get_servo_addr_32, ex: H104 I{id} D{addr}
                servo_id = gcode_p.get_id_num(command, default=0)
                addr = gcode_p.get_addr(command)
                ret = self.get_servo_addr_32(servo_id=servo_id, addr=addr)
            elif num == 105:  # H105 set_servo_zero, ex: H105 I{id}
                servo_id = gcode_p.get_id_num(command, default=0)
                ret = self.set_servo_zero(servo_id=servo_id)
            elif num == 106:  # H106 get_servo_debug_msg, ex: H106
                ret = self.get_servo_debug_msg()
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
            return ret

        def __handle_gcode_m(num):
            if num == 116:  # M116 set_gripper_enable, ex: M116 V{enable}
                value = gcode_p.get_int_value(command)
                ret = self.set_gripper_enable(value)
            elif num == 117:  # M117 set_gripper_mode, ex: M117 V{mode}
                value = gcode_p.get_int_value(command)
                ret = self.set_gripper_mode(value)
            elif num == 118:  # M118 set_gripper_zero, ex: M118
                ret = self.set_gripper_zero()
            elif num == 119:  # M119 get_gripper_position, ex: M119
                ret = self.get_gripper_position()
            elif num == 120:  # M120 set_gripper_position, ex: M120 V{pos}
                value = gcode_p.get_int_value(command)
                ret = self.set_gripper_position(value)
            elif num == 121:  # M121 set_gripper_speed, ex: M121 V{speed}
                value = gcode_p.get_int_value(command)
                ret = self.set_gripper_speed(value)
            elif num == 125:  # M125 get_gripper_err_code, ex: M125
                ret = self.get_gripper_err_code()
            elif num == 126:  # M126 clean_gripper_error, ex: M126
                ret = self.clean_gripper_error()
            elif num == 127:
                ret = self.get_gripper_version()
            elif num == 131:  # M131 get_tgpio_digital, ex: M131
                ret = self.get_tgpio_digital()
            elif num == 132:  # M132 set_tgpio_digital, ex: M132 I{ionum} V{}
                ionum = gcode_p.get_id_num(command, default=0)
                value = gcode_p.get_int_value(command)
                ret = self.set_tgpio_digital(ionum, value)
            elif num == 133:  # M133 get_tgpio_analog(0), ex: M133 I{ionum=0}
                ionum = gcode_p.get_id_num(command, default=0)
                ret = self.get_tgpio_analog(ionum=ionum)
            elif num == 134:  # M134 get_tgpio_analog(1), ex: M134 I{ionum=1}
                ionum = gcode_p.get_id_num(command, default=0)
                ret = self.get_tgpio_analog(ionum=ionum)
            elif num == 135:
                return self.get_tgpio_version()
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
            return ret

        def __handle_gcode_d(num):
            if num == 11:  # D11 I{id}
                id_num = gcode_p.get_id_num(command, default=None)
                ret = self.get_servo_error_code(id_num)
            elif num == 12:  # D12 I{id}
                id_num = gcode_p.get_id_num(command, default=None)
                if id_num == 0:
                    id_num = 8
                self.clean_error()
                self.clean_warn()
                self.motion_enable(enable=False, servo_id=id_num)
                ret = self.set_servo_detach(id_num)
            elif num == 13:  # D13 I{id}
                id_num = gcode_p.get_id_num(command, default=None)
                if id_num == 0:
                    id_num = 8
                self.set_servo_zero(id_num)
                ret = self.motion_enable(enable=True, servo_id=id_num)
            elif num == 21:  # D21 I{id}
                id_num = gcode_p.get_id_num(command, default=None)
                self.clean_servo_pvl_err(id_num)
                ret = self.get_servo_error_code(id_num)
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
            return ret

        def __handle_gcode_s(num):
            if num == 44:  # S44 I{id}
                id_num = gcode_p.get_id_num(command, default=None)
                ret = self.get_servo_all_pids(id_num)
            elif num == 45:
                id_num = gcode_p.get_id_num(command, default=1)
                ret = self.get_servo_version(servo_id=id_num)
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
            return ret

        def __handle_gcode_c(num):
            if num == 131:  # C131 get_cgpio_digital, ex: C131
                ret = self.get_cgpio_digital()
            elif num == 132:  # C132 get_cgpio_analog(0), ex: C132 I{ionum=0}
                ionum = gcode_p.get_id_num(command, default=0)
                ret = self.get_cgpio_analog(ionum)
            elif num == 133:  # C133 get_cgpio_analog(1), ex: C133 I{ionum=1}
                ionum = gcode_p.get_id_num(command, default=1)
                ret = self.get_cgpio_analog(ionum)
            elif num == 134:  # C134 set_cgpio_digital, ex: C134 I{ionum} V{value}
                ionum = gcode_p.get_id_num(command, default=0)
                value = gcode_p.get_int_value(command)
                ret = self.set_cgpio_digital(ionum, value)
            elif num == 135:  # C135 set_cgpio_analog(0, v), ex: C135 I{ionum=0} V{value}
                ionum = gcode_p.get_id_num(command, default=0)
                value = gcode_p.get_float_value(command)
                ret = self.set_cgpio_analog(ionum, value)
            elif num == 136:  # C136 set_cgpio_analog(1, v), ex: C136 I{ionum=1} V{value}
                ionum = gcode_p.get_id_num(command, default=1)
                value = gcode_p.get_float_value(command)
                ret = self.set_cgpio_analog(ionum, value)
            elif num == 137:  # C137 set_cgpio_digital_input_function, ex: C137 I{ionum} V{fun}
                ionum = gcode_p.get_id_num(command, default=0)
                value = gcode_p.get_int_value(command)
                ret = self.set_cgpio_digital_input_function(ionum, value)
            elif num == 138:  # C138 set_cgpio_digital_output_function, ex: C138 I{ionum} V{fun}
                ionum = gcode_p.get_id_num(command, default=0)
                value = gcode_p.get_int_value(command)
                ret = self.set_cgpio_digital_output_function(ionum, value)
            elif num == 139:  # C139 get_cgpio_state, ex: C139
                ret = self.get_cgpio_state()
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
            return ret

        cmd_num = gcode_p.get_gcode_cmd_num(command, 'G')
        if cmd_num >= 0:
            return __handle_gcode_g(cmd_num)
        cmd_num = gcode_p.get_gcode_cmd_num(command, 'H')
        if cmd_num >= 0:
            return __handle_gcode_h(cmd_num)
        cmd_num = gcode_p.get_gcode_cmd_num(command, 'M')
        if cmd_num >= 0:
            return __handle_gcode_m(cmd_num)
        cmd_num = gcode_p.get_gcode_cmd_num(command, 'D')
        if cmd_num >= 0:
            return __handle_gcode_d(cmd_num)
        cmd_num = gcode_p.get_gcode_cmd_num(command, 'S')
        if cmd_num >= 0:
            return __handle_gcode_s(cmd_num)
        cmd_num = gcode_p.get_gcode_cmd_num(command, 'C')
        if cmd_num >= 0:
            return __handle_gcode_c(cmd_num)
        logger.debug('command {} is not exist'.format(command))
        return APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)

    @xarm_is_connected(_type='set')
    def run_gcode_file(self, path, **kwargs):
        times = kwargs.get('times', 1)
        init = kwargs.get('init', False)
        mode = kwargs.get('mode', 0)
        state = kwargs.get('state', 0)
        wait_seconds = kwargs.get('wait_seconds', 0)
        try:
            abs_path = os.path.abspath(path)
            if not os.path.exists(abs_path):
                raise FileNotFoundError
            with open(abs_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            lines = [line.strip() for line in lines]
            if init:
                self.clean_error()
                self.clean_warn()
                self.motion_enable(True)
                self.set_mode(mode)
                self.set_state(state)
            if wait_seconds > 0:
                time.sleep(wait_seconds)

            for i in range(times):
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    if not self.connected:
                        logger.error('xArm is disconnect')
                        return APIState.NOT_CONNECTED
                    ret = self.send_cmd_sync(line)
                    if isinstance(ret, int) and ret < 0:
                        return ret
            return APIState.NORMAL
        except Exception as e:
            logger.error(e)
            return APIState.API_EXCEPTION

    @xarm_is_connected(_type='set')
    def run_blockly_app(self, path, **kwargs):
        """
        Run the app generated by xArmStudio software
        :param path: app path
        """
        try:
            if not os.path.exists(path):
                path = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'projects', 'test', 'xarm{}'.format(self.axis), 'app', 'myapp', path)
            if os.path.isdir(path):
                path = os.path.join(path, 'app.xml')
            if not os.path.exists(path):
                raise FileNotFoundError
            blockly_tool = BlocklyTool(path)
            succeed = blockly_tool.to_python(arm=self._api_instance, is_exec=True, **kwargs)
            if succeed:
                times = kwargs.get('times', 1)
                highlight_callback = kwargs.get('highlight_callback', None)
                blockly_print = kwargs.get('blockly_print', print)
                connect_changed_callbacks = self._report_callbacks[self.REPORT_CONNECT_CHANGED_ID].copy()
                state_changed_callbacks = self._report_callbacks[self.REPORT_STATE_CHANGED_ID].copy()
                error_warn_changed_callbacks = self._report_callbacks[self.REPORT_ERROR_WARN_CHANGED_ID].copy()
                count_changed_callbacks = self._report_callbacks[self.REPORT_COUNT_CHANGED_ID].copy()
                code = APIState.NORMAL
                try:
                    for _ in range(times):
                        exec(blockly_tool.codes, {'arm': self._api_instance, 'highlight_callback': highlight_callback, 'print': blockly_print})
                except Exception as e:
                    code = APIState.RUN_BLOCKLY_EXCEPTION
                    blockly_print('run blockly app error: {}'.format(e))
                self._report_callbacks[self.REPORT_CONNECT_CHANGED_ID] = connect_changed_callbacks
                self._report_callbacks[self.REPORT_STATE_CHANGED_ID] = state_changed_callbacks
                self._report_callbacks[self.REPORT_ERROR_WARN_CHANGED_ID] = error_warn_changed_callbacks
                self._report_callbacks[self.REPORT_COUNT_CHANGED_ID] = count_changed_callbacks
                return code
            else:
                logger.error('The conversion is incomplete and some blocks are not yet supported.')
                return APIState.CONVERT_FAILED
        except Exception as e:
            logger.error(e)
            return APIState.API_EXCEPTION

    @xarm_is_connected(_type='get')
    def get_hd_types(self):
        ret = self.arm_cmd.get_hd_types()
        return ret[0], ret[1:]

    @xarm_is_connected(_type='set')
    def reload_dynamics(self):
        ret = self.arm_cmd.reload_dynamics()
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> reload_dynamics -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_counter_reset(self):
        ret = self.arm_cmd.cnter_reset()
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_counter_reset -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_counter_increase(self, val=1):
        ret = self.arm_cmd.cnter_plus()
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_counter_increase -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_report_tau_or_i(self, tau_or_i=0):
        ret = self.arm_cmd.set_report_tau_or_i(int(tau_or_i))
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_report_tau_or_i({}) -> code={}'.format(tau_or_i, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_report_tau_or_i(self):
        ret = self.arm_cmd.get_report_tau_or_i()
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_self_collision_detection(self, on_off):
        ret = self.arm_cmd.set_self_collision_detection(int(on_off))
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_self_collision_detection({}) -> code={}'.format(on_off, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_collision_tool_model(self, tool_type, *args, **kwargs):
        if tool_type == XCONF.CollisionToolType.BOX:
            assert ('z' in kwargs or len(args) >= 3) \
                and ('y' in kwargs or len(args) >= 2) \
                and ('x' in kwargs or len(args) >= 1), 'params error, must specify x,y,z parameter'
            x = kwargs.get('x') if 'x' in kwargs else args[0]
            y = kwargs.get('y') if 'y' in kwargs else args[1]
            z = kwargs.get('z') if 'z' in kwargs else args[2]
            params = [x, y, z]
        elif tool_type == XCONF.CollisionToolType.CYLINDER:
            assert ('radius' in kwargs or len(args) >= 2) \
                   and ('height' in kwargs or len(args) >= 1), 'params error, must specify radius,height parameter'
            radius = kwargs.get('radius') if 'radius' in kwargs else args[0]
            height = kwargs.get('height') if 'height' in kwargs else args[1]
            params = [radius, height]
        else:
            params = [] if tool_type < XCONF.CollisionToolType.USE_PRIMITIVES else list(args)
        ret = self.arm_cmd.set_collision_tool_model(tool_type, params)
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_collision_tool_model({}, {}) -> code={}'.format(tool_type, params, ret[0]), code=ret[0])
        return ret[0]

    def get_firmware_config(self):
        cgpio_code, cgpio_states = self.get_cgpio_state()
        reduced_code, reduced_states = self.get_reduced_states()
        tau_code, tau_flag = self.get_report_tau_or_i()
        code = cgpio_code if reduced_code == 0 and tau_code == 0 else reduced_code if cgpio_code == 0 and tau_code == 0 else tau_code
        return code, {
            'COLL_SENS': self.collision_sensitivity,  # 碰撞灵敏度
            'TEACH_SENS': self.teach_sensitivity,  # 示教灵敏度
            'GRAV_DIR': self.gravity_direction,  # 重力方向
            'TCP_LOAD': self.tcp_load,  # TCP负载
            'TCP_OFFSET': self.position_offset,  # TCP偏移
            'TCP_MAXACC': self.tcp_acc_limit[1],  # TCP的最大加速度
            'TCP_JERK': self.tcp_jerk,  # TCP加加速度
            'JOINT_MAXACC': self.joint_acc_limit[1],  # 关节的最大加速度
            'JOINT_JERK': self.joint_jerk,  # 关节加加速度
            'WORLD_OFFSET': self.world_offset,  # 基坐标偏移
            'REPORT_TAU_OR_I': tau_flag,  # 上报力矩还是电流
            'CGPIO_INPUT_FUNC_CONFIG': cgpio_states[10],  # 控制器数字输入IO的配置功能
            'CGPIO_OUTPUT_FUNC_CONFIG': cgpio_states[11],  # 控制器数字输出IO的配置功能
            'REDUCED_STATES': reduced_states,  # 缩减模式的状态
            'GPIO_RESET_CONFIG': self.gpio_reset_config,  # gpio自动复位配置
            'COLL_PARAMS': self.self_collision_params,  # 碰撞模型参数
        }

    def set_firmware_config(self, config):
        code, old_config = self.get_firmware_config()
        if 'COLL_SENS' in config and config['COLL_SENS'] != old_config['COLL_SENS']:
            self.set_collision_sensitivity(config['COLL_SENS'])
        if 'TEACH_SENS' in config and config['TEACH_SENS'] != old_config['TEACH_SENS']:
            self.set_teach_sensitivity(config['TEACH_SENS'])
        if 'GRAV_DIR' in config and config['GRAV_DIR'] != old_config['GRAV_DIR']:
            self.set_gravity_direction(config['GRAV_DIR'])
        if 'TCP_LOAD' in config and config['TCP_LOAD'] != old_config['TCP_LOAD']:
            self.set_tcp_load(*config['TCP_LOAD'])
        if 'TCP_OFFSET' in config and config['TCP_OFFSET'] != old_config['TCP_OFFSET']:
            self.set_tcp_offset(config['TCP_OFFSET'])
        if 'TCP_MAXACC' in config and config['TCP_MAXACC'] != old_config['TCP_MAXACC']:
            self.set_tcp_maxacc(config['TCP_MAXACC'])
        if 'TCP_JERK' in config and config['TCP_JERK'] != old_config['TCP_JERK']:
            self.set_tcp_jerk(config['TCP_JERK'])
        if 'JOINT_MAXACC' in config and config['JOINT_MAXACC'] != old_config['JOINT_MAXACC']:
            self.set_joint_maxacc(config['JOINT_MAXACC'])
        if 'JOINT_JERK' in config and config['JOINT_JERK'] != old_config['JOINT_JERK']:
            self.set_joint_jerk(config['JOINT_JERK'])
        if 'WORLD_OFFSET' in config and config['WORLD_OFFSET'] != old_config['WORLD_OFFSET']:
            self.set_world_offset(config['WORLD_OFFSET'])
        if 'REPORT_TAU_OR_I' in config and config['REPORT_TAU_OR_I'] != old_config['REPORT_TAU_OR_I']:
            self.set_report_tau_or_i(config['REPORT_TAU_OR_I'])
        if 'GPIO_RESET_CONFIG' in config and config['GPIO_RESET_CONFIG'] != old_config['GPIO_RESET_CONFIG']:
            self.config_io_reset_when_stop(0, config['GPIO_RESET_CONFIG'][0])
            self.config_io_reset_when_stop(1, config['GPIO_RESET_CONFIG'][1])

        if 'REDUCED_STATES' in config:
            states = config['REDUCED_STATES']
            old_states = old_config['REDUCED_STATES']
            if states[1] != old_states[1]:
                self.set_reduced_tcp_boundary(states[1])
            if states[2] != old_states[2]:
                self.set_reduced_max_tcp_speed(states[2])
            if states[3] != old_states[3]:
                self.set_reduced_max_joint_speed(states[3])
            if len(states) > 4 and len(old_states) > 4:
                if states[4] != old_states[4]:
                    self.set_reduced_joint_range(states[4])
            if len(states) > 5 and len(old_states) > 5:
                if states[5] != old_states[5]:
                    self.set_fense_mode(states[5])
            if len(states) > 6 and len(old_states) > 6:
                if states[4] != old_states[6]:
                    self.set_collision_rebound(states[6])
            self.set_reduced_mode(states[0])

        if 'CGPIO_INPUT_FUNC_CONFIG' in config and config['CGPIO_INPUT_FUNC_CONFIG'] != old_config['CGPIO_INPUT_FUNC_CONFIG']:
            for i in range(len(config['CGPIO_INPUT_FUNC_CONFIG'])):
                if config['CGPIO_INPUT_FUNC_CONFIG'][i] != old_config['CGPIO_INPUT_FUNC_CONFIG'][i]:
                    self.set_cgpio_digital_input_function(i, config['CGPIO_INPUT_FUNC_CONFIG'][i])
        if 'CGPIO_OUTPUT_FUNC_CONFIG' in config and config['CGPIO_OUTPUT_FUNC_CONFIG'] != old_config['CGPIO_OUTPUT_FUNC_CONFIG']:
            for i in range(len(config['CGPIO_OUTPUT_FUNC_CONFIG'])):
                if config['CGPIO_OUTPUT_FUNC_CONFIG'][i] != old_config['CGPIO_OUTPUT_FUNC_CONFIG'][i]:
                    self.set_cgpio_digital_output_function(i, config['CGPIO_OUTPUT_FUNC_CONFIG'][i])

        if 'COLL_PARAMS' in config and config['COLL_PARAMS'] != old_config['COLL_PARAMS']:
            if config['COLL_PARAMS'][0] != old_config['COLL_PARAMS'][0]:
                self.set_self_collision_detection(config['COLL_PARAMS'][0])
            if config['COLL_PARAMS'][1] != old_config['COLL_PARAMS'][1] or config['COLL_PARAMS'][2] != old_config['COLL_PARAMS'][2]:
                self.set_collision_tool_model(config['COLL_PARAMS'][1], *config['COLL_PARAMS'][2])

        self.save_conf()

    @xarm_is_connected(_type='get')
    def get_power_board_version(self):
        ret = self.arm_cmd.get_power_board_version()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1:]

    @xarm_is_connected(_type='get')
    def get_movement(self):
        ret = self.arm_cmd.get_movement()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1]
    
    @xarm_is_connected(_type='set')
    def vc_set_joint_velocity(self, speeds, is_radian=None, is_sync=True, check_mode=True, duration=-1):
        # if check_mode and not self._check_mode_is_correct(4):
        #     return APIState.MODE_IS_NOT_CORRECT
        is_radian = self._default_is_radian if is_radian is None else is_radian
        jnt_v = [0] * 7
        for i, spd in enumerate(speeds):
            if i >= 7:
                break
            jnt_v[i] = to_radian(spd, is_radian)

        ret = self.arm_cmd.vc_set_jointv(jnt_v, 1 if is_sync else 0, duration if self.version_is_ge(1, 8, 0) else -1)
        ret[0] = self._check_code(ret[0], is_move_cmd=True, mode=4)
        self.log_api_info('API -> vc_set_joint_velocity -> code={}, speeds={}, is_sync={}'.format(
            ret[0], jnt_v, is_sync
        ), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def vc_set_cartesian_velocity(self, speeds, is_radian=None, is_tool_coord=False, check_mode=True, duration=-1):
        # if check_mode and not self._check_mode_is_correct(5):
        #     return APIState.MODE_IS_NOT_CORRECT
        is_radian = self._default_is_radian if is_radian is None else is_radian
        line_v = [0] * 6
        for i, spd in enumerate(speeds):
            if i >= 6:
                break
            line_v[i] = spd if i <= 2 else to_radian(spd, is_radian)
        ret = self.arm_cmd.vc_set_linev(line_v, 1 if is_tool_coord else 0, duration if self.version_is_ge(1, 8, 0) else -1)
        ret[0] = self._check_code(ret[0], is_move_cmd=True, mode=5)
        self.log_api_info('API -> vc_set_cartesian_velocity -> code={}, speeds={}, is_tool_coord={}'.format(
            ret[0], line_v, is_tool_coord
        ), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def calibrate_tcp_coordinate_offset(self, four_points, is_radian=None):
        assert len(four_points) >= 4, 'The parameter four_points must contain 4 TCP points'
        is_radian = self._default_is_radian if is_radian is None else is_radian
        points = []
        for i in range(4):
            assert len(four_points[i]) >= 6, 'Each TCP point in the parameter four_points must contain x/y/z/roll/pitch/yaw'
            points.append([four_points[i][j] if j <= 2 else to_radian(four_points[i][j], is_radian) for j in range(6)])
        ret = self.arm_cmd.cali_tcp_pose(points)
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1:]

    @xarm_is_connected(_type='get')
    def calibrate_tcp_orientation_offset(self, rpy_be, rpy_bt, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        rpy_be_ = [to_radian(rpy_be[i], input_is_radian) for i in range(3)]
        rpy_bt_ = [to_radian(rpy_bt[i], input_is_radian) for i in range(3)]
        ret = self.arm_cmd.cali_tcp_orient(rpy_be_, rpy_bt_)
        ret[0] = self._check_code(ret[0])
        return ret[0], [ret[i+1] if return_is_radian else math.degrees(ret[i+1]) for i in range(3)]

    @xarm_is_connected(_type='get')
    def calibrate_user_orientation_offset(self, three_points, mode=0, trust_ind=0, input_is_radian=None, return_is_radian=None):
        assert len(three_points) >= 3, 'The parameter three_points must contain 3 TCP points'
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        points = []
        for i in range(3):
            assert len(three_points[i]) >= 6, 'Each TCP point in the parameter three_points must contain x/y/z/roll/pitch/yaw'
            points.append([three_points[i][j] if j <= 2 else to_radian(three_points[i][j], input_is_radian) for j in range(6)])
        ret = self.arm_cmd.cali_user_orient(points, mode=mode, trust_ind=trust_ind)
        ret[0] = self._check_code(ret[0])
        return ret[0], [ret[i+1] if return_is_radian else math.degrees(ret[i+1]) for i in range(3)]

    @xarm_is_connected(_type='get')
    def calibrate_user_coordinate_offset(self, rpy_ub, pos_b_uorg, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        rpy_ub_ = [rpy_ub[i] if is_radian else math.radians(rpy_ub[i]) for i in range(3)]
        ret = self.arm_cmd.cali_user_pos(rpy_ub_, pos_b_uorg)
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1:4]

    @xarm_is_connected(_type='set')
    def get_tcp_rotation_radius(self, value=6):
        ret = self.arm_cmd.get_tcp_rotation_radius(value)
        self.log_api_info('API -> get_tcp_rotation_radius -> code={}'.format(ret[0]), code=ret[0])
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1][0]

    @xarm_is_connected(_type='set')
    def get_max_joint_velocity(self, eveloc, joint_pos, is_radian=None):
        """
        Obtain maximum joint angular velocity
        :param eveloc: Maximum TCP speed
        :param joint_pos: joint angle list (unit: rad if is_radian is True else °), angle should be a list of values
            whose length is the number of joints like [axis-1, axis-2, axis-3, axis-3, axis-4, axis-5, axis-6, axis-7]
        :param is_radian: the max_joint_speed of the states is in radians or not, default is self.default_is_radian
        """
        is_radian = self._default_is_radian if is_radian is None else is_radian
        joints = [0] * 7
        for i in range(min(len(joint_pos), 7)):
            joints[i] = to_radian(joint_pos[i], is_radian)
        return self.arm_cmd.get_max_joint_velocity(eveloc, joints)

    @xarm_is_connected(_type='get')
    def iden_tcp_load(self, estimated_mass=0):
        protocol_identifier = self.arm_cmd.get_protocol_identifier()
        self.arm_cmd.set_protocol_identifier(2)
        self._keep_heart = False
        if self.version_is_ge(1, 9, 100) and estimated_mass <= 0:
            estimated_mass = 0.5
        ret = self.arm_cmd.iden_tcp_load(estimated_mass)
        self.arm_cmd.set_protocol_identifier(protocol_identifier)
        self._keep_heart = True
        self.log_api_info('API -> iden_tcp_load -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0]), ret[1:5]

    @xarm_is_connected(_type='set')
    def set_cartesian_velo_continuous(self, on_off):
        ret = self.arm_cmd.set_cartesian_velo_continuous(int(on_off))
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_cartesian_velo_continuous({}) -> code={}'.format(on_off, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_allow_approx_motion(self, on_off):
        ret = self.arm_cmd.set_allow_approx_motion(int(on_off))
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_allow_approx_motion({}) -> code={}'.format(on_off, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_allow_approx_motion(self):
        ret = self.arm_cmd.get_allow_approx_motion()
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> get_allow_approx_motion() -> code={}'.format(ret[0]), code=ret[0])
        return ret[0], ret[-1]
    
    @xarm_is_connected(_type='get')
    def iden_joint_friction(self, sn=None):
        if sn is None:
            code, sn = self.get_robot_sn()
            if code != 0:
                self.log_api_info('iden_joint_friction -> get_robot_sn failed, code={}'.format(code), code=code)
                return APIState.API_EXCEPTION, -1
        if len(sn) != 14:
            self.log_api_info('iden_joint_friction, sn is not correct, sn={}'.format(sn), code=APIState.API_EXCEPTION)
            return APIState.API_EXCEPTION, -1
        sn = sn.upper()
        axis_map = {5: 'F', 6: 'I', 7: 'S'}
        valid_850 = self.is_850 and sn[0] == 'F' and sn[1] == 'X'
        valid_lite = self.is_lite6 and sn[0] == 'L' and sn[1] == 'I'
        valid_xarm = not self.is_850 and not self.is_lite6 and sn[0] == 'X' and sn[1] == axis_map.get(self.axis, '')
        if not (valid_850 or valid_lite or valid_xarm):
            self.log_api_info('iden_joint_friction, sn is not correct, axis={}, type={}, sn={}'.format(self.axis, self.device_type, sn), code=APIState.API_EXCEPTION)
            return APIState.API_EXCEPTION, -1

        protocol_identifier = self.arm_cmd.get_protocol_identifier()
        self.arm_cmd.set_protocol_identifier(2)
        self._keep_heart = False
        ret = self.arm_cmd.iden_joint_friction(sn)
        self.arm_cmd.set_protocol_identifier(protocol_identifier)
        self._keep_heart = True
        self.log_api_info('API -> iden_joint_friction -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0]), 0 if int(ret[1]) == 0 else -1

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def wait_all_task_finish(self, timeout=None, **kwargs):
        if not self._support_feedback:
            return APIState.CMD_NOT_EXIST
        wait = kwargs.pop('wait', True)
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        ret = self.arm_cmd.check_feedback(feedback_key=feedback_key)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0])
        if wait and ret[0] == 0:
            ret[0] = self._wait_feedback(timeout, trans_id=trans_id, ignore_log=True)[0]
            if ret[0] == 0:
                time.sleep(0.5)
        return ret[0]

    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def send_hex_cmd(self, datas, timeout=10):
        ret = self.arm_cmd.send_hex_cmd(datas, timeout)
        return ret[1:]
        # ret = self.arm_cmd.send_hex_request(datas)
        # if ret == -1:
        #     return [XCONF.UxbusState.ERR_NOTTCP]
        # ret = self.arm_cmd.recv_hex_request(ret, timeout)
        # return ret