#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import re
import math
import time
import threading
from collections import Iterable
from ..core.comm import SerialPort, SocketPort
from ..core.config.x_config import XCONF
from ..core.wrapper import UxbusCmdSer, UxbusCmdTcp
from ..core.utils import convert
from ..core.utils.log import logger, pretty_print
from ..core.config.x_code import ControllerWarn, ControllerError
from .gripper import Gripper
from .gpio import GPIO
from .servo import Servo
from .events import *
from . import parse
from .code import APIState
from .utils import xarm_is_connected, xarm_is_ready, compare_time, compare_version

RAD_DEGREE = 57.295779513082320876798154814105


class XArm(Gripper, Servo, GPIO, Events):
    def __init__(self, port=None, is_radian=False, do_not_open=False, **kwargs):
        super(XArm, self).__init__()
        self._port = port
        self._baudrate = kwargs.get('baudrate', XCONF.SerialConf.SERIAL_BAUD)
        self._timeout = kwargs.get('timeout', None)
        self._filters = kwargs.get('filters', None)
        self._enable_heartbeat = kwargs.get('enable_heartbeat', False)
        self._enable_report = kwargs.get('enable_report', True)
        self._report_type = kwargs.get('report_type', 'rich')

        self._check_tcp_limit = kwargs.get('check_tcp_limit', True)
        self._check_joint_limit = kwargs.get('check_joint_limit', True)
        self._check_cmdnum_limit = kwargs.get('check_cmdnum_limit', True)

        self._min_tcp_speed, self._max_tcp_speed = 0.1, 1000  # mm/s
        self._min_tcp_acc, self._max_tcp_acc = 1.0, 50000  # mm/s^2
        self._tcp_jerk = 1000  # mm/s^3

        self._min_joint_speed, self._max_joint_speed = 0.01, 4.0  # rad/s
        self._min_joint_acc, self._max_joint_acc = 0.01, 20.0  # rad/s^2
        self._joint_jerk = 20.0  # rad/s^3

        self._rot_jerk = 2.3
        self._max_rot_acc = 2.7

        self._stream_type = 'serial'
        self._stream = None
        self.arm_cmd = None
        self._stream_report = None
        self._report_thread = None
        self._only_report_err_warn_changed = True

        self._last_position = [201.5, 0, 140.5, 3.1415926, 0, 0]  # [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
        self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [servo_1(rad), servo_2(rad), servo_3(rad), servo_4(rad), servo_5(rad), servo_6(rad), servo_7(rad)]
        self._last_tcp_speed = 100  # mm/s, rad/s
        self._last_tcp_acc = 2000  # mm/s^2, rad/s^2
        self._last_joint_speed = 0.3490658503988659  # 20 °/s
        self._last_joint_acc = 8.726646259971648  # 500 °/s^2
        self._mvtime = 0

        self._version = None
        self._robot_sn = None
        self._position = [201.5, 0, 140.5, 3.1415926, 0, 0]
        self._angles = [0] * 7
        self._position_offset = [0] * 6
        self._state = 4
        self._mode = 0
        self._joints_torque = [0, 0, 0, 0, 0, 0, 0]  # 力矩
        self._tcp_load = [0, [0, 0, 0]]  # 负载[重量, 重心], [weight, [x, y, z]]
        self._collision_sensitivity = 0  # 碰撞灵敏度
        self._teach_sensitivity = 0  # 示教灵敏度
        self._error_code = 0
        self._warn_code = 0
        self._cmd_num = 0
        self._arm_type = XCONF.Robot.Type.XARM7_X4
        self._arm_axis = XCONF.Robot.Axis.XARM7
        self._arm_master_id = 0
        self._arm_slave_id = 0
        self._arm_motor_tid = 0
        self._arm_motor_fid = 0
        self._arm_motor_brake_states = [0, 0, 0, 0, 0, 0, 0, 0]  # [motor-1-brake-state, ..., motor-7-brake, reserved]
        self._arm_motor_enable_states = [0, 0, 0, 0, 0, 0, 0, 0]  # [motor-1-enable-state, ..., motor-7-enable, reserved]
        self._gravity_direction = [0, 0, -1]

        self._is_ready = False
        self._is_stop = False
        self._is_sync = False
        self._is_first_report = True
        self._default_is_radian = is_radian

        self._sleep_finish_time = time.time()
        self._is_old_protocol = False

        self._major_version_number = 0  # 固件主版本号
        self._minor_version_number = 0  # 固件次版本号
        self._revision_version_number = 0  # 固件修正版本号

        Events.__init__(self)
        if not do_not_open:
            self.connect()

    @property
    def version_number(self):
        return self._major_version_number, self._minor_version_number, self._revision_version_number

    @property
    def connected(self):
        return self._stream and self._stream.connected

    @property
    def ready(self):
        return self._is_ready

    @property
    def default_is_radian(self):
        return self._default_is_radian

    @property
    def version(self):
        if not self._version:
            self.get_version()
        return 'v' + '.'.join(map(str, self.version_number))

    @property
    def sn(self):
        return self._robot_sn

    @property
    def position(self):
        if not self._enable_report:
            self.get_position()
        return [self._position[i] * RAD_DEGREE if 2 < i < 6 and not self._default_is_radian
                else self._position[i] for i in range(len(self._position))]

    @property
    def tcp_speed_limit(self):
        return [self._min_tcp_speed, self._max_tcp_speed]

    @property
    def tcp_acc_limit(self):
        return [self._min_tcp_acc, self._max_tcp_acc]

    @property
    def last_used_position(self):
        return [self._last_position[i] * RAD_DEGREE if 2 < i < 6 and not self._default_is_radian
                else self._last_position[i] for i in range(len(self._last_position))]

    @property
    def last_used_tcp_speed(self):
        return self._last_tcp_speed

    @property
    def last_used_tcp_acc(self):
        return self._last_tcp_acc

    @property
    def angles(self):
        if not self._enable_report:
            self.get_servo_angle()
        return [angle if self._default_is_radian else angle * RAD_DEGREE for angle in self._angles]

    @property
    def joint_speed_limit(self):
        limit = [self._min_joint_speed, self._max_joint_speed]
        if not self._default_is_radian:
            limit = [i * RAD_DEGREE for i in limit]
        return limit

    @property
    def joint_acc_limit(self):
        limit = [self._min_joint_acc, self._max_joint_acc]
        if not self._default_is_radian:
            limit = [i * RAD_DEGREE for i in limit]
        return limit

    @property
    def last_used_angles(self):
        return [angle if self._default_is_radian else angle * RAD_DEGREE for angle in self._last_angles]

    @property
    def last_used_joint_speed(self):
        return self._last_joint_speed if self._default_is_radian else self._last_joint_speed * RAD_DEGREE

    @property
    def last_used_joint_acc(self):
        return self._last_joint_acc if self._default_is_radian else self._last_joint_acc * RAD_DEGREE

    @property
    def position_offset(self):
        return [self._position_offset[i] * RAD_DEGREE if 2 < i < 6 and not self._default_is_radian
                else self._position_offset[i] for i in range(len(self._position_offset))]

    @property
    def state(self):
        if not self._enable_report:
            self.get_state()
        return self._state

    @property
    def mode(self):
        return self._mode

    @property
    def joints_torque(self):
        return self._joints_torque

    @property
    def tcp_load(self):
        return self._tcp_load

    @property
    def collision_sensitivity(self):
        return self._collision_sensitivity

    @property
    def teach_sensitivity(self):
        return self._teach_sensitivity

    @property
    def motor_brake_states(self):
        return self._arm_motor_brake_states

    @property
    def motor_enable_states(self):
        return self._arm_motor_enable_states

    @property
    def error_code(self):
        if not self._enable_report:
            self.get_err_warn_code()
        return self._error_code

    @property
    def warn_code(self):
        if not self._enable_report:
            self.get_err_warn_code()
        return self._warn_code

    @property
    def has_error(self):
        return self.error_code != 0

    @property
    def has_warn(self):
        return self.warn_code != 0

    @property
    def has_err_warn(self):
        return self.has_error or self._warn_code != 0 or (self.arm_cmd and self.arm_cmd.has_err_warn)

    @property
    def cmd_num(self):
        if not self._enable_report:
            self.get_cmdnum()
        return self._cmd_num

    @property
    def device_type(self):
        return self._arm_type

    @property
    def axis(self):
        return self._arm_axis

    @property
    def master_id(self):
        return self._arm_master_id

    @property
    def slave_id(self):
        return self._arm_slave_id

    @property
    def motor_tid(self):
        return self._arm_motor_tid

    @property
    def motor_fid(self):
        return self._arm_motor_fid

    @property
    def gravity_direction(self):
        return self._gravity_direction

    def connect(self, port=None, baudrate=None, timeout=None):
        if self.connected:
            return
        self._is_ready = True
        self._port = port if port is not None else self._port
        self._baudrate = baudrate if baudrate is not None else self._baudrate
        self._timeout = timeout if timeout is not None else self._timeout
        if not self._port:
            raise Exception('can not connect to port/ip {}'.format(self._port))
        if isinstance(self._port, (str, bytes)):
            if self._port == 'localhost' or re.match(
                    r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$",
                    self._port):
                self._stream = SocketPort(self._port, XCONF.SocketConf.TCP_CONTROL_PORT,
                                          heartbeat=self._enable_heartbeat,
                                          buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE)
                if not self.connected:
                    raise Exception('connect socket failed')

                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdTcp(self._stream)
                self._stream_type = 'socket'
                self._check_version()

                try:
                    self._connect_report()
                except:
                    self._stream_report = None

                if self._stream.connected and self._enable_report:
                    if self._is_old_protocol:
                        self._report_thread = threading.Thread(target=self._report_thread_handle_old, daemon=True)
                    else:
                        self._report_thread = threading.Thread(target=self._report_thread_handle, daemon=True)
                    self._report_thread.start()
                self._report_connect_changed_callback()
            else:
                self._stream = SerialPort(self._port)
                if not self.connected:
                    raise Exception('connect serail failed')

                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdSer(self._stream)
                self._stream_type = 'serial'
                self._check_version()
                if self._enable_report:
                    self._report_thread = threading.Thread(target=self._auto_get_report_thread, daemon=True)
                    self._report_thread.start()
                    self._report_connect_changed_callback(True, True)
                else:
                    self._report_connect_changed_callback(True, False)

    def _check_version(self):
        self._version = None
        try:
            count = 30
            while not self._version and count:
                self.get_version()
                time.sleep(0.1)
                count -= 1
            pattern = re.compile(r'.*[vV](\d+)\.(\d+)\.(\d+)$')
            m = re.match(pattern, self._version)
            if m:
                self._major_version_number, self._minor_version_number, self._revision_version_number = map(int,
                                                                                                            m.groups())
            else:
                version_date = '-'.join(self._version.split('-')[-3:])
                self._is_old_protocol = compare_time('2019-02-01', version_date)
                if self._is_old_protocol:
                    self._major_version_number = 0
                    self._minor_version_number = 0
                    self._revision_version_number = 1
                else:
                    self._major_version_number = 0
                    self._minor_version_number = 1
                    self._revision_version_number = 0
            count = 5
            while not self._robot_sn and count and self.warn_code == 0:
                self.get_robot_sn()
                self.get_err_warn_code()
                time.sleep(0.1)
                count -= 1
            if self.warn_code != 0:
                self.clean_warn()
            print('is_old_protocol: {}'.format(self._is_old_protocol))
            print('version_number: {}.{}.{}'.format(self._major_version_number, self._minor_version_number,
                                                    self._revision_version_number))
        except Exception as e:
            print('compare_time: {}, {}'.format(self._version, e))

    def _connect_report(self):
        if self._enable_report:
            if self._stream_report:
                try:
                    self._stream_report.close()
                except:
                    pass
                time.sleep(2)
            if self._report_type == 'real':
                self.__connect_report_real()
            elif self._report_type == 'normal':
                self.__connect_report_normal()
            else:
                self.__connect_report_rich()

    def __connect_report_normal(self):
        if self._stream_type == 'socket':
            self._stream_report = SocketPort(self._port,
                                             XCONF.SocketConf.TCP_REPORT_NORM_PORT,
                                             buffer_size=XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE
                                             if not self._is_old_protocol else 87)

    def __connect_report_rich(self):
        if self._stream_type == 'socket':
            self._stream_report = SocketPort(self._port,
                                             XCONF.SocketConf.TCP_REPORT_RICH_PORT,
                                             buffer_size=XCONF.SocketConf.TCP_REPORT_RICH_BUF_SIZE
                                             if not self._is_old_protocol else 187)

    def __connect_report_real(self):
        if self._stream_type == 'socket':
            self._stream_report = SocketPort(self._port,
                                             XCONF.SocketConf.TCP_REPORT_REAL_PORT,
                                             buffer_size=XCONF.SocketConf.TCP_REPORT_REAL_BUF_SIZE
                                             if not self._is_old_protocol else 87)

    def _report_connect_changed_callback(self, main_connected=None, report_connected=None):
        if REPORT_CONNECT_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_CONNECT_CHANGED_ID]:
                try:
                    callback({
                        'connected': self._stream and self._stream.connected if main_connected is None else main_connected,
                        'reported': self._stream_report and self._stream_report.connected if report_connected is None else report_connected,
                    })
                except Exception as e:
                    logger.error('connect changed callback: {}'.format(e))

    def _report_state_changed_callback(self):
        if REPORT_STATE_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_STATE_CHANGED_ID]:
                try:
                    callback({
                        'state': self._state
                    })
                except Exception as e:
                    logger.error('state changed callback: {}'.format(e))

    def _report_mode_changed_callback(self):
        if REPORT_MODE_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_MODE_CHANGED_ID]:
                try:
                    callback({
                        'mode': self._mode
                    })
                except Exception as e:
                    logger.error('mode changed callback: {}'.format(e))

    def _report_mtable_mtbrake_changed_callback(self):
        if REPORT_MTABLE_MTBRAKE_CHANGED_ID in self._report_callbacks.keys():
            mtable = [bool(i) for i in self._arm_motor_enable_states]
            mtbrake = [bool(i) for i in self._arm_motor_brake_states]
            for callback in self._report_callbacks[REPORT_MTABLE_MTBRAKE_CHANGED_ID]:
                try:
                    callback({
                        'mtable': mtable.copy(),
                        'mtbrake': mtbrake.copy()
                    })
                except Exception as e:
                    logger.error('mtable/mtbrake changed callback: {}'.format(e))

    def _report_error_warn_changed_callback(self):
        if REPORT_ERROR_WARN_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_ERROR_WARN_CHANGED_ID]:
                try:
                    callback({
                        'warn_code': self._warn_code,
                        'error_code': self._error_code,
                    })
                except Exception as e:
                    logger.error('error warn changed callback: {}'.format(e))

    def _report_cmdnum_changed_callback(self):
        if REPORT_CMDNUM_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_CMDNUM_CHANGED_ID]:
                try:
                    callback({
                        'cmdnum': self._cmd_num,
                    })
                except Exception as e:
                    logger.error('cmdnum changed callback: {}'.format(e))

    def _report_location_callback(self):
        if REPORT_LOCATION_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[REPORT_LOCATION_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self._position.copy()
                if item['joints']:
                    ret['joints'] = self._angles.copy()
                try:
                    callback(ret)
                except Exception as e:
                    logger.error('location callback: {}'.format(e))

    def _report_callback(self):
        if REPORT_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[REPORT_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self._position.copy()
                if item['joints']:
                    ret['joints'] = self._angles.copy()
                if item['error_code']:
                    ret['error_code'] = self._error_code
                if item['warn_code']:
                    ret['warn_code'] = self._warn_code
                if item['state']:
                    ret['state'] = self._state
                if item['mtable']:
                    mtable = [bool(i) for i in self._arm_motor_enable_states]
                    ret['mtable'] = mtable.copy()
                if item['mtbrake']:
                    mtbrake = [bool(i) for i in self._arm_motor_brake_states]
                    ret['mtbrake'] = mtbrake.copy()
                if item['cmdnum']:
                    ret['cmdnum'] = self._cmd_num
                try:
                    callback(ret)
                except Exception as e:
                    logger.error('report callback: {}'.format(e))

    def _report_thread_handle_old(self):
        def __handle_report_normal(rx_data):
            # print('length:', convert.bytes_to_u32(rx_data[0:4]))
            state, mtbrake, mtable, error_code, warn_code = rx_data[4:9]
            angles = convert.bytes_to_fp32s(rx_data[9:7 * 4 + 9], 7)
            pose = convert.bytes_to_fp32s(rx_data[37:6 * 4 + 37], 6)
            cmd_num = convert.bytes_to_u16(rx_data[61:63])
            pose_offset = convert.bytes_to_fp32s(rx_data[63:6 * 4 + 63], 6)

            if error_code != self._error_code or warn_code != self._warn_code:
                self._report_error_warn_changed_callback()
                if error_code != self._error_code:
                    self._error_code = error_code
                    if self._error_code != 0:
                        pretty_print('Error, Code: {}'.format(self._error_code), color='red')
                    else:
                        pretty_print('Error had clean', color='blue')
                if warn_code != self._warn_code:
                    self._warn_code = warn_code
                    if self._warn_code != 0:
                        pretty_print('WarnCode: {}'.format(self._warn_code), color='yellow')
                    else:
                        pretty_print('Warnning had clean', color='blue')
            elif not self._only_report_err_warn_changed:
                self._report_error_warn_changed_callback()

            if cmd_num != self._cmd_num:
                self._cmd_num = cmd_num
                self._report_cmdnum_changed_callback()

            if state != self._state:
                self._state = state
                self._report_state_changed_callback()

            mtbrake = [mtbrake & 0x01, mtbrake >> 1 & 0x01, mtbrake >> 2 & 0x01, mtbrake >> 3 & 0x01,
                       mtbrake >> 4 & 0x01, mtbrake >> 5 & 0x01, mtbrake >> 6 & 0x01, mtbrake >> 7 & 0x01]
            mtable = [mtable & 0x01, mtable >> 1 & 0x01, mtable >> 2 & 0x01, mtable >> 3 & 0x01,
                      mtable >> 4 & 0x01, mtable >> 5 & 0x01, mtable >> 6 & 0x01, mtable >> 7 & 0x01]

            if mtbrake != self._arm_motor_brake_states or mtable != self._arm_motor_enable_states:
                self._arm_motor_enable_states = mtable
                self._arm_motor_brake_states = mtbrake
                self._report_mtable_mtbrake_changed_callback()

            if not self._is_first_report:
                if state == 4 or not all([bool(item[0] & item[1]) for item in zip(mtbrake, mtable)][:self.axis]):
                    if self._is_ready:
                        logger.info('[report], xArm is not ready to move', color='orange')
                    self._is_ready = False
                else:
                    if not self._is_ready:
                        logger.info('[report], xArm is ready to move', color='green')
                    self._is_ready = True
            else:
                self._is_ready = False
            self._is_first_report = False

            self._error_code = error_code
            self._warn_code = warn_code
            self.arm_cmd.has_err_warn = error_code != 0 or warn_code != 0
            self._state = state
            self._cmd_num = cmd_num
            self._arm_motor_brake_states = mtbrake
            self._arm_motor_enable_states = mtable

            for i in range(len(pose)):
                if i < 3:
                    pose[i] = float('{:.3f}'.format(pose[i]))
                    # pose[i] = float('{:.3f}'.format(pose[i][0]))
                else:
                    pose[i] = float('{:.6f}'.format(pose[i]))
                    # pose[i] = float('{:.6f}'.format(pose[i][0]))
            for i in range(len(angles)):
                angles[i] = float('{:.6f}'.format(angles[i]))
                # angles[i] = float('{:.6f}'.format(angles[i][0]))
            for i in range(len(pose_offset)):
                if i < 3:
                    pose_offset[i] = float('{:.3f}'.format(pose_offset[i]))
                    # pose_offset[i] = float('{:.3f}'.format(pose_offset[i][0]))
                else:
                    pose_offset[i] = float('{:.6f}'.format(pose_offset[i]))
                    # pose_offset[i] = float('{:.6f}'.format(pose_offset[i][0]))

            if math.inf not in pose and -math.inf not in pose and not (10 <= self._error_code <= 17):
                self._position = pose
            if math.inf not in angles and -math.inf not in angles and not (10 <= self._error_code <= 17):
                self._angles = angles
            if math.inf not in pose_offset and -math.inf not in pose_offset and not (10 <= self._error_code <= 17):
                self._position_offset = pose_offset

            self._report_location_callback()

            self._report_callback()
            if not self._is_sync:
                self._sync()
                self._is_sync = True

        def __handle_report_rich(rx_data):
            __handle_report_normal(rx_data)
            (self._arm_type,
             arm_axis,
             self._arm_master_id,
             self._arm_slave_id,
             self._arm_motor_tid,
             self._arm_motor_fid) = rx_data[87:93]

            if 7 >= arm_axis >= 5:
                self._arm_axis = arm_axis
            if self._arm_type == 5:
                self._arm_axis = 5
            elif self._arm_type == 6:
                self._arm_axis = 6
            elif self._arm_type == 3:
                self._arm_axis = 7

            ver_msg = rx_data[93:122]
            # self._version = str(ver_msg, 'utf-8')

            trs_msg = convert.bytes_to_fp32s(rx_data[123:143], 5)
            # trs_msg = [i[0] for i in trs_msg]
            (self._tcp_jerk,
             self._min_tcp_acc,
             self._max_tcp_acc,
             self._min_tcp_speed,
             self._max_tcp_speed) = trs_msg
            # print('tcp_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._tcp_jerk, self._min_tcp_acc, self._max_tcp_acc, self._min_tcp_speed, self._max_tcp_speed
            # ))

            p2p_msg = convert.bytes_to_fp32s(rx_data[143:163], 5)
            # p2p_msg = [i[0] for i in p2p_msg]
            (self._joint_jerk,
             self._min_joint_acc,
             self._max_joint_acc,
             self._min_joint_speed,
             self._max_joint_speed) = p2p_msg
            # print('joint_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._joint_jerk, self._min_joint_acc, self._max_joint_acc,
            #     self._min_joint_speed, self._max_joint_speed
            # ))

            rot_msg = convert.bytes_to_fp32s(rx_data[163:171], 2)
            # rot_msg = [i[0] for i in rot_msg]
            self._rot_jerk, self._max_rot_acc = rot_msg
            # print('rot_jerk: {}, mac_acc: {}'.format(self._rot_jerk, self._max_rot_acc))

            sv3_msg = convert.bytes_to_u16s(rx_data[171:187], 8)

        main_socket_connected = self._stream and self._stream.connected
        report_socket_connected = self._stream_report and self._stream_report.connected
        while self._stream and self._stream.connected:
            try:
                if not self._stream_report or not self._stream_report.connected:
                    self.get_err_warn_code()
                    if report_socket_connected:
                        report_socket_connected = False
                        self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                    self._connect_report()
                    continue
                if not report_socket_connected:
                    report_socket_connected = True
                    self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                rx_data = self._stream_report.read()
                if rx_data != -1 and len(rx_data) >= 87:
                    if len(rx_data) == 87:
                        __handle_report_normal(rx_data)
                    elif len(rx_data) >= 187:
                        __handle_report_rich(rx_data)
            except Exception as e:
                logger.error(e)
            time.sleep(0.001)
        self.disconnect()
        self._report_connect_changed_callback(False, False)

    def _report_thread_handle(self):
        def __handle_report_normal(rx_data):
            # print('length:', convert.bytes_to_u32(rx_data[0:4]))
            state, mode = rx_data[4] & 0x0F, rx_data[4] >> 4
            cmd_num = convert.bytes_to_u16(rx_data[5:7])
            angles = convert.bytes_to_fp32s(rx_data[7:7 * 4 + 7], 7)
            pose = convert.bytes_to_fp32s(rx_data[35:6 * 4 + 35], 6)
            torque = convert.bytes_to_fp32s(rx_data[59:7 * 4 + 59], 7)
            mtbrake, mtable, error_code, warn_code = rx_data[87:91]
            pose_offset = convert.bytes_to_fp32s(rx_data[91:6 * 4 + 91], 6)
            tcp_load = convert.bytes_to_fp32s(rx_data[115:4 * 4 + 115], 4)
            collis_sens, teach_sens = rx_data[131:133]
            self._gravity_direction = convert.bytes_to_fp32s(rx_data[133:3*4 + 133], 3)

            # print('torque: {}'.format(torque))
            # print('tcp_load: {}'.format(tcp_load))
            # print('collis_sens: {}, teach_sens: {}'.format(collis_sens, teach_sens))

            if error_code != self._error_code or warn_code != self._warn_code:
                self._report_error_warn_changed_callback()
                if error_code != self._error_code:
                    self._error_code = error_code
                    if self._error_code != 0:
                        pretty_print('Error, Code: {}'.format(self._error_code), color='red')
                    else:
                        pretty_print('Error had clean', color='blue')
                if warn_code != self._warn_code:
                    self._warn_code = warn_code
                    if self._warn_code != 0:
                        pretty_print('WarnCode: {}'.format(self._warn_code), color='yellow')
                    else:
                        pretty_print('Warnning had clean', color='blue')
            elif not self._only_report_err_warn_changed:
                self._report_error_warn_changed_callback()

            if cmd_num != self._cmd_num:
                self._cmd_num = cmd_num
                self._report_cmdnum_changed_callback()

            if state != self._state:
                self._state = state
                self._report_state_changed_callback()
            if mode != self._mode:
                self._mode = mode
                self._report_mode_changed_callback()

            mtbrake = [mtbrake & 0x01, mtbrake >> 1 & 0x01, mtbrake >> 2 & 0x01, mtbrake >> 3 & 0x01,
                       mtbrake >> 4 & 0x01, mtbrake >> 5 & 0x01, mtbrake >> 6 & 0x01, mtbrake >> 7 & 0x01]
            mtable = [mtable & 0x01, mtable >> 1 & 0x01, mtable >> 2 & 0x01, mtable >> 3 & 0x01,
                      mtable >> 4 & 0x01, mtable >> 5 & 0x01, mtable >> 6 & 0x01, mtable >> 7 & 0x01]

            if mtbrake != self._arm_motor_brake_states or mtable != self._arm_motor_enable_states:
                self._arm_motor_enable_states = mtable
                self._arm_motor_brake_states = mtbrake
                self._report_mtable_mtbrake_changed_callback()

            if not self._is_first_report:
                if state == 4 or not all([bool(item[0] & item[1]) for item in zip(mtbrake, mtable)][:self.axis]):
                    if self._is_ready:
                        logger.info('[report], xArm is not ready to move', color='orange')
                    self._is_ready = False
                else:
                    if not self._is_ready:
                        logger.info('[report], xArm is ready to move', color='green')
                    self._is_ready = True
            else:
                self._is_ready = False
            self._is_first_report = False

            self._error_code = error_code
            self._warn_code = warn_code
            self.arm_cmd.has_err_warn = error_code != 0 or warn_code != 0
            self._state = state
            self._mode = mode
            self._cmd_num = cmd_num
            self._arm_motor_brake_states = mtbrake
            self._arm_motor_enable_states = mtable
            self._joints_torque = torque
            if compare_version(self.version_number, (0, 2, 0)):
                self._tcp_load = [tcp_load[0], [i for i in tcp_load[1:]]]
            else:
                self._tcp_load = [tcp_load[0], [i * 1000 for i in tcp_load[1:]]]
            self._collision_sensitivity = collis_sens
            self._teach_sensitivity = teach_sens

            for i in range(len(pose)):
                if i < 3:
                    pose[i] = float('{:.3f}'.format(pose[i]))
                    # pose[i] = float('{:.3f}'.format(pose[i][0]))
                else:
                    pose[i] = float('{:.6f}'.format(pose[i]))
                    # pose[i] = float('{:.6f}'.format(pose[i][0]))
            for i in range(len(angles)):
                angles[i] = float('{:.6f}'.format(angles[i]))
                # angles[i] = float('{:.6f}'.format(angles[i][0]))
            for i in range(len(pose_offset)):
                if i < 3:
                    pose_offset[i] = float('{:.3f}'.format(pose_offset[i]))
                    # pose_offset[i] = float('{:.3f}'.format(pose_offset[i][0]))
                else:
                    pose_offset[i] = float('{:.6f}'.format(pose_offset[i]))
                    # pose_offset[i] = float('{:.6f}'.format(pose_offset[i][0]))

            if math.inf not in pose and -math.inf not in pose and not (10 <= self._error_code <= 17):
                self._position = pose
            if math.inf not in angles and -math.inf not in angles and not (10 <= self._error_code <= 17):
                self._angles = angles
            if math.inf not in pose_offset and -math.inf not in pose_offset and not (10 <= self._error_code <= 17):
                self._position_offset = pose_offset

            self._report_location_callback()

            self._report_callback()
            if not self._is_sync:
                self._sync()
                self._is_sync = True

        def __handle_report_rich(rx_data):
            __handle_report_normal(rx_data)
            (self._arm_type,
             arm_axis,
             self._arm_master_id,
             self._arm_slave_id,
             self._arm_motor_tid,
             self._arm_motor_fid) = rx_data[145:151]

            if 7 >= arm_axis >= 5:
                self._arm_axis = arm_axis

            # self._version = str(rx_data[151:180], 'utf-8')

            trs_msg = convert.bytes_to_fp32s(rx_data[181:201], 5)
            # trs_msg = [i[0] for i in trs_msg]
            (self._tcp_jerk,
             self._min_tcp_acc,
             self._max_tcp_acc,
             self._min_tcp_speed,
             self._max_tcp_speed) = trs_msg
            # print('tcp_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._tcp_jerk, self._min_tcp_acc, self._max_tcp_acc, self._min_tcp_speed, self._max_tcp_speed
            # ))

            p2p_msg = convert.bytes_to_fp32s(rx_data[201:221], 5)
            # p2p_msg = [i[0] for i in p2p_msg]
            (self._joint_jerk,
             self._min_joint_acc,
             self._max_joint_acc,
             self._min_joint_speed,
             self._max_joint_speed) = p2p_msg
            # print('joint_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._joint_jerk, self._min_joint_acc, self._max_joint_acc,
            #     self._min_joint_speed, self._max_joint_speed
            # ))

            rot_msg = convert.bytes_to_fp32s(rx_data[221:229], 2)
            # rot_msg = [i[0] for i in rot_msg]
            self._rot_jerk, self._max_rot_acc = rot_msg
            # print('rot_jerk: {}, mac_acc: {}'.format(self._rot_jerk, self._max_rot_acc))

            sv3_msg = rx_data[229:245]

        main_socket_connected = self._stream and self._stream.connected
        report_socket_connected = self._stream_report and self._stream_report.connected
        while self._stream and self._stream.connected:
            try:
                if not self._stream_report or not self._stream_report.connected:
                    self.get_err_warn_code()
                    if report_socket_connected:
                        report_socket_connected = False
                        self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                    self._connect_report()
                    continue
                if not report_socket_connected:
                    report_socket_connected = True
                    self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                rx_data = self._stream_report.read()
                if rx_data != -1 and len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                    # if len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                    #     __handle_report_normal(rx_data)
                    if len(rx_data) >= XCONF.SocketConf.TCP_REPORT_RICH_BUF_SIZE:
                        __handle_report_rich(rx_data)
                    else:
                        __handle_report_normal(rx_data)
            except Exception as e:
                logger.error(e)
            time.sleep(0.001)
        self.disconnect()
        self._report_connect_changed_callback(False, False)

    def _auto_get_report_thread(self):
        logger.debug('get report thread start')
        while self.connected:
            try:
                cmd_num = self._cmd_num
                state = self._state
                error_code = self._error_code
                warn_code = self._warn_code

                self.get_cmdnum()
                time.sleep(0.01)
                self.get_state()
                time.sleep(0.01)
                self.get_err_warn_code()
                time.sleep(0.01)
                self.get_servo_angle()
                time.sleep(0.01)
                self.get_position()

                if cmd_num != self._cmd_num:
                    self._report_cmdnum_changed_callback()
                if state != self._state:
                    self._report_state_changed_callback()
                if state == 4:
                    if self._is_ready:
                        logger.info('[report], xArm is not ready to move', color='orange')
                    self._is_ready = False
                else:
                    if not self._is_ready:
                        logger.info('[report], xArm is ready to move', color='green')
                    self._is_ready = True
                if error_code != self._error_code or warn_code != self._warn_code:
                    self._report_error_warn_changed_callback()
                elif not self._only_report_err_warn_changed and (self._error_code != 0 or self._warn_code != 0):
                    self._report_error_warn_changed_callback()

                self._report_location_callback()
                self._report_callback()

                if self._cmd_num >= XCONF.MAX_CMD_NUM:
                    time.sleep(1)

                time.sleep(0.1)
            except:
                pass
        self._report_connect_changed_callback(False, False)
        logger.debug('get report thread stopped')

    def disconnect(self):
        self._stream.close()
        if self._stream_report:
            try:
                self._stream_report.close()
            except:
                pass
        self._is_ready = False
        try:
            self._stream.join()
        except:
            pass
        if self._stream_report:
            try:
                self._stream_report.join()
            except:
                pass
        self._report_connect_changed_callback(False, False)

    def _sync(self):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_position = self._position
        self._last_angles = self._angles

    class _WaitMove:
        def __init__(self, owner, timeout):
            self.owner = owner
            self.timeout = timeout if timeout is not None else 10
            self.timer = None
            self.is_timeout = False

        def start(self):
            if self.timeout > 0:
                self.timer = threading.Timer(self.timeout, self.timeout_cb)
                self.timer.setDaemon(True)
                self.timer.start()
            self.check_stop_move()

        def check_stop_move(self):
            base_joint_pos = self.owner.angles.copy()
            time.sleep(0.1)
            count = 0
            while not self.is_timeout and not self.owner._is_stop and self.owner.connected and not self.owner.has_error:
                if time.time() < self.owner._sleep_finish_time:
                    time.sleep(0.01)
                    continue
                if self.owner.angles == base_joint_pos or self.owner.state != 1:
                    count += 1
                    if count >= 6:
                        break
                else:
                    base_joint_pos = self.owner._angles.copy()
                    count = 0
                time.sleep(0.05)
            # if not self.is_timeout:
            #     self.owner._sync()

        def timeout_cb(self):
            self.is_timeout = True

    @xarm_is_connected(_type='get')
    def get_position(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_tcp_pose()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 6:
            # self._position = [float('{:.6f}'.format(ret[i][0])) for i in range(1, 7)]
            self._position = [float('{:.6f}'.format(ret[i])) for i in range(1, 7)]
            ret[0] = 0
        return ret[0], [self._position[i] * RAD_DEGREE if 2 < i < 6 and not is_radian else self._position[i] for i in
                        range(len(self._position))]

    def _is_out_of_tcp_range(self, value, i):
        if not self._check_tcp_limit or self._stream_type != 'socket' or not self._enable_report:
            return False
        tcp_range = XCONF.Robot.TCP_LIMITS.get(self.axis).get(self.device_type, [])
        if 2 < i < len(tcp_range):  # only limit rotate
            limit = tcp_range[i]
            if limit[0] == limit[1]:
                if value == limit[0] or value == limit[0] - 2 * math.pi:
                    return False
                else:
                    return True
            if value < limit[0] or value > limit[1]:
                return True
        return False

    def _wait_until_cmdnum_lt_max(self):
        if not self._check_cmdnum_limit:
            return
        self._is_stop = False
        while self.cmd_num >= XCONF.MAX_CMD_NUM:
            if not self.connected:
                return APIState.NOT_CONNECTED
            elif not self.ready:
                return APIState.NOT_READY
            elif self._is_stop:
                return APIState.EMERGENCY_STOP
            elif self.has_error:
                return
            time.sleep(0.2)

    @xarm_is_ready(_type='set')
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None,
                     wait=False, timeout=None, **kwargs):
        ret = self._wait_until_cmdnum_lt_max()
        if ret is not None:
            return ret

        is_radian = self._default_is_radian if is_radian is None else is_radian
        tcp_pos = [x, y, z, roll, pitch, yaw]
        last_used_position = self._last_position.copy()
        last_used_tcp_speed = self._last_tcp_speed
        last_used_tcp_acc = self._last_tcp_acc
        for i in range(6):
            value = tcp_pos[i]
            if value is None:
                continue
            elif isinstance(value, str):
                if value.isdigit():
                    value = float(value)
                else:
                    continue
            if relative:
                if 2 < i < 6:
                    if is_radian:
                        if self._is_out_of_tcp_range(self._last_position[i] + value, i):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] += value
                    else:
                        if self._is_out_of_tcp_range(self._last_position[i] + value / RAD_DEGREE, i):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] += value / RAD_DEGREE
                else:
                    self._last_position[i] += value
            else:
                if 2 < i < 6:
                    if is_radian:
                        if self._is_out_of_tcp_range(value, i):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] = value
                    else:
                        if self._is_out_of_tcp_range(value / RAD_DEGREE, i):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] = value / RAD_DEGREE
                else:
                    self._last_position[i] = value

        if speed is not None:
            if isinstance(speed, str):
                if speed.isdigit():
                    speed = float(speed)
                else:
                    speed = self._last_tcp_speed
            self._last_tcp_speed = min(max(speed, self._min_tcp_speed), self._max_tcp_speed)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                if mvvelo.isdigit():
                    mvvelo = float(mvvelo)
                else:
                    mvvelo = self._last_tcp_speed
            self._last_tcp_speed = min(max(mvvelo, self._min_tcp_speed), self._max_tcp_speed)
        if mvacc is not None:
            if isinstance(mvacc, str):
                if mvacc.isdigit():
                    mvacc = float(mvacc)
                else:
                    mvacc = self._last_tcp_acc
            self._last_tcp_acc = min(max(mvacc, self._min_tcp_acc), self._max_tcp_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                if mvacc.isdigit():
                    mvtime = float(mvtime)
                else:
                    mvtime = self._mvtime
            self._mvtime = mvtime

        if kwargs.get('check', False):
            _, limit = self.is_tcp_limit(self._last_position)
            if _ == 0 and limit is True:
                self._last_position = last_used_position
                self._last_tcp_speed = last_used_tcp_speed
                self._last_tcp_acc = last_used_tcp_acc
                return APIState.TCP_LIMIT
        if radius is not None and radius >= 0:
            ret = self.arm_cmd.move_lineb(self._last_position, self._last_tcp_speed, self._last_tcp_acc, self._mvtime, radius)
            if ret[0] != 0:
                logger.debug('exception({}): move arc line: pos={}, mvvelo={}, mvacc={}, mvtime={}, mvradius={}'.format(
                    ret[0], self._last_position, self._last_tcp_speed, self._last_tcp_acc, self._mvtime, radius
                ))
            else:
                logger.debug('move arc line: {}, mvvelo={}, mvacc={}, mvtime={}, mvradius={}'.format(
                    self._last_position, self._last_tcp_speed, self._last_tcp_acc, self._mvtime, radius
                ))
        else:
            ret = self.arm_cmd.move_line(self._last_position, self._last_tcp_speed, self._last_tcp_acc, self._mvtime)
            if ret[0] != 0:
                logger.debug('exception({}): move line: pos={}, mvvelo={}, mvacc={}, mvtime={}'.format(
                    ret[0], self._last_position, self._last_tcp_speed, self._last_tcp_acc, self._mvtime
                ))
            else:
                logger.debug('move line: {}, mvvelo={}, mvacc={}, mvtime={}'.format(
                    self._last_position, self._last_tcp_speed, self._last_tcp_acc, self._mvtime
                ))
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
        if ret[0] < 0 and not self.get_is_moving():
            self._last_position = last_used_position
            self._last_tcp_speed = last_used_tcp_speed
            self._last_tcp_acc = last_used_tcp_acc
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_angle(self, servo_id=None, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_joint_pos()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 7:
            # self._angles = [float('{:.6f}'.format(ret[i][0])) for i in range(1, 8)]
            self._angles = [float('{:.6f}'.format(ret[i])) for i in range(1, 8)]
            ret[0] = 0
        if servo_id is None or servo_id == 8 or len(self._angles) < servo_id:
            return ret[0], list(map(lambda x: x if is_radian else x * RAD_DEGREE, self._angles))
        else:
            return ret[0], self._angles[servo_id-1] if is_radian else self._angles[servo_id-1] * RAD_DEGREE

    def _is_out_of_joint_range(self, angle, i):
        if not self._check_joint_limit or self._stream_type != 'socket' or not self._enable_report:
            return False
        joint_limit = XCONF.Robot.JOINT_LIMITS.get(self.axis).get(self.device_type, [])
        if i < len(joint_limit):
            angle_range = joint_limit[i]
            if angle <= angle_range[0] or angle >= angle_range[1]:
                return True
        return False

    @xarm_is_ready(_type='set')
    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=None, wait=False, timeout=None, **kwargs):
        assert ((servo_id is None or servo_id == 8) and isinstance(angle, Iterable)) \
            or (1 <= servo_id <= 7 and angle is not None and not isinstance(angle, Iterable)), \
            'param servo_id or angle error'
        ret = self._wait_until_cmdnum_lt_max()
        if ret is not None:
            return ret

        last_used_angle = self._last_angles.copy()
        last_used_joint_speed = self._last_joint_speed
        last_used_joint_acc = self._last_joint_acc
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if servo_id is None or servo_id == 8:
            for i in range(min(len(angle), len(self._last_angles))):
                value = angle[i]
                if value is None or i >= self.axis:
                    continue
                if isinstance(value, str):
                    if value.isdigit():
                        value = float(value)
                    else:
                        continue
                if relative:
                    if is_radian:
                        if self._is_out_of_joint_range(self._last_angles[i] + value, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] += value
                    else:
                        if self._is_out_of_joint_range(self._last_angles[i] + value / RAD_DEGREE, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] += value / RAD_DEGREE
                else:
                    if is_radian:
                        if self._is_out_of_joint_range(value, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] = value
                    else:
                        if self._is_out_of_joint_range(value / RAD_DEGREE, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] = value / RAD_DEGREE
        else:
            if servo_id > self.axis:
                return APIState.SERVO_NOT_EXIST
            if isinstance(angle, str):
                if angle.isdigit():
                    angle = float(angle)
                else:
                    raise Exception('param angle error')
            if relative:
                if is_radian:
                    if self._is_out_of_joint_range(self._last_angles[servo_id - 1] + angle, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] += angle
                else:
                    if self._is_out_of_joint_range(self._last_angles[servo_id - 1] + angle / RAD_DEGREE, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] += angle / RAD_DEGREE
            else:
                if is_radian:
                    if self._is_out_of_joint_range(angle, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] = angle
                else:
                    if self._is_out_of_joint_range(angle / RAD_DEGREE, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] = angle / RAD_DEGREE

        if speed is not None:
            if isinstance(speed, str):
                if speed.isdigit():
                    speed = float(speed)
                else:
                    speed = self._last_joint_speed if is_radian else self._last_joint_speed * RAD_DEGREE
            if not is_radian:
                speed /= RAD_DEGREE
            self._last_joint_speed = min(max(speed, self._min_joint_speed), self._max_joint_speed)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                if mvvelo.isdigit():
                    mvvelo = float(mvvelo)
                else:
                    mvvelo = self._last_joint_speed if is_radian else self._last_joint_speed * RAD_DEGREE
            if not is_radian:
                mvvelo /= RAD_DEGREE
            self._last_joint_speed = min(max(mvvelo, self._min_joint_speed), self._max_joint_speed)
        if mvacc is not None:
            if isinstance(mvacc, str):
                if mvacc.isdigit():
                    mvacc = float(mvacc)
                else:
                    mvacc = self._last_joint_acc if is_radian else self._last_joint_acc * RAD_DEGREE
            if not is_radian:
                mvacc /= RAD_DEGREE
            self._last_joint_acc = min(max(mvacc, self._min_joint_acc), self._max_joint_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                if mvacc.isdigit():
                    mvtime = float(mvtime)
                else:
                    mvtime = self._mvtime
            self._mvtime = mvtime

        if kwargs.get('check', False):
            _, limit = self.is_joint_limit(self._last_angles)
            if _ == 0 and limit is True:
                self._last_angles = last_used_angle
                self._last_joint_speed = last_used_joint_speed
                self._last_joint_acc = last_used_joint_acc
                return APIState.JOINT_LIMIT

        ret = self.arm_cmd.move_joint(self._last_angles, self._last_joint_speed, self._last_joint_acc, self._mvtime)
        if ret[0] != 0:
            logger.debug('exception({}): move joint: joint={}, mvvelo={}, mvacc={}, mvtime={}'.format(
                ret[0], self._last_angles, self._last_joint_speed, self._last_joint_acc, self._mvtime
            ))
        else:
            logger.debug('move joint: {}, mvvelo={}, mvacc={}, mvtime={}'.format(
                self._last_angles, self._last_joint_speed, self._last_joint_acc, self._mvtime
            ))
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
        if ret[0] < 0 and not self.get_is_moving():
            self._last_angles = last_used_angle
            self._last_joint_speed = last_used_joint_speed
            self._last_joint_acc = last_used_joint_acc
        return ret[0]

    @xarm_is_ready(_type='set')
    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if not is_radian:
            angles = [angle / RAD_DEGREE for angle in angles]
        for i in range(self.axis):
            if self._is_out_of_joint_range(angles[i], i):
                return APIState.OUT_OF_RANGE
        ret = self.arm_cmd.move_servoj(angles, self._last_joint_speed, self._last_joint_acc, self._mvtime)
        return ret[0]

    @xarm_is_ready(_type='set')
    def move_circle(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        last_used_tcp_speed = self._last_tcp_speed
        last_used_tcp_acc = self._last_tcp_acc
        is_radian = self._default_is_radian if is_radian is None else is_radian
        pose_1 = []
        pose_2 = []
        for i in range(6):
            pose_1.append(pose1[i] if i < 3 or is_radian else pose1[i] / RAD_DEGREE)
            pose_2.append(pose2[i] if i < 3 or is_radian else pose2[i] / RAD_DEGREE)
        if speed is not None:
            if isinstance(speed, str):
                if speed.isdigit():
                    speed = float(speed)
                else:
                    speed = self._last_tcp_speed
            self._last_tcp_speed = min(max(speed, self._min_tcp_speed), self._max_tcp_speed)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                if mvvelo.isdigit():
                    mvvelo = float(mvvelo)
                else:
                    mvvelo = self._last_tcp_speed
            self._last_tcp_speed = min(max(mvvelo, self._min_tcp_speed), self._max_tcp_speed)
        if mvacc is not None:
            if isinstance(mvacc, str):
                if mvacc.isdigit():
                    mvacc = float(mvacc)
                else:
                    mvacc = self._last_tcp_acc
            self._last_tcp_acc = min(max(mvacc, self._min_tcp_acc), self._max_tcp_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                if mvacc.isdigit():
                    mvtime = float(mvtime)
                else:
                    mvtime = self._mvtime
            self._mvtime = mvtime

        ret = self.arm_cmd.move_circle(pose_1, pose_2, self._last_tcp_speed, self._last_tcp_acc, self._mvtime, percent)
        if ret[0] != 0:
            logger.debug('exception({}): move circle: pos1={}, pos2={}, percent={}'.format(
                ret[0], pose_1, pose_2, percent
            ))
        else:
            logger.debug('move circle: pos1={}, pos2={}, percent={}'.format(
                pose_1, pose_2, percent
            ))

        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
        if ret[0] < 0 and not self.get_is_moving():
            self._last_tcp_speed = last_used_tcp_speed
            self._last_tcp_acc = last_used_tcp_acc
        return ret[0]

    @xarm_is_ready(_type='set')
    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if speed is None:
            speed = 0.8726646259971648  # 50 °/s
        else:
            if not is_radian:
                speed /= RAD_DEGREE
        if mvacc is None:
            mvacc = 17.453292519943297  # 1000 °/s^2
        else:
            if not is_radian:
                mvacc /= RAD_DEGREE
        if mvtime is None:
            mvtime = 0

        ret = self.arm_cmd.move_gohome(speed, mvacc, mvtime)
        if ret[0] != 0:
            logger.debug('exception({}): move gohome: mvvelo={}, mvacc={}, mvtime={}'.format(
                ret[0], speed, mvacc, mvtime
            ))
        else:
            logger.debug('move gohome: mvvelo={}, mvacc={}, mvtime={}'.format(
                speed, mvacc, mvtime
            ))
        if ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            self._last_position = [201.5, 0, 140.5, -3.1415926, 0, 0]
            self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
        return ret[0]

    @xarm_is_ready(_type='set')
    def move_arc_lines(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0,
                       automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):
        assert len(paths) > 0, 'parameter paths error'
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if speed is None:
            speed = self._last_tcp_speed
        if mvacc is None:
            mvacc = self._last_tcp_acc
        if mvtime is None:
            mvtime = 0

        if automatic_calibration:
            _ = self.set_position(*paths[0], is_radian=is_radian, speed=speed, mvacc=mvacc, mvtime=mvtime, wait=True)
            if _ < 0:
                logger.error('quit, api failed, code={}'.format(_))
                return
            _, angles = self.get_servo_angle(is_radian=True)
        self.set_pause_time(first_pause_time)
        self._is_stop = False
        last_used_joint_speed = self._last_joint_speed

        def _move():
            if automatic_calibration:
                ret = self.set_servo_angle(angle=angles, is_radian=True, speed=0.8726646259971648, wait=False)
                if ret < 0:
                    logger.error('set_servo_angle, ret={}'.format(ret))
                    return -1
                self._last_joint_speed = last_used_joint_speed
            for path in paths:
                if len(path) > 6 and path[6] >= 0:
                    radius = path[6]
                else:
                    radius = 0
                if self.has_error or self._is_stop:
                    return -2
                ret = self.set_position(*path[:6], radius=radius, is_radian=is_radian, wait=False, speed=speed, mvacc=mvacc, mvtime=mvtime)
                if ret < 0:
                    logger.error('set_positon, ret={}'.format(ret))
                    return -1
            return 0
        count = 1
        api_failed = False

        def state_changed_callback(item):
            if item['state'] == 4:
                self._is_stop = True

        self.register_state_changed_callback(state_changed_callback)
        try:
            if times == 0:
                while not self.has_error and not self._is_stop:
                    _ = _move()
                    if _ == -1:
                        api_failed = True
                        break
                    elif _ == -2:
                        break
                    count += 1
                    self.set_pause_time(repeat_pause_time)
                if api_failed:
                    logger.error('quit, api error')
                elif self._error_code != 0:
                    logger.error('quit, controller error')
                elif self._is_stop:
                    logger.error('quit, emergency_stop')
            else:
                for i in range(times):
                    if self.has_error or self._is_stop:
                        break
                    _ = _move()
                    if _ == -1:
                        api_failed = True
                        break
                    elif _ == -2:
                        break
                    count += 1
                    self.set_pause_time(repeat_pause_time)
                if api_failed:
                    logger.error('quit, api error')
                elif self._error_code != 0:
                    logger.error('quit, controller error')
                elif self._is_stop:
                    logger.error('quit, emergency_stop')
        except:
            pass
        finally:
            self.release_state_changed_callback(state_changed_callback)
        if wait:
            self._WaitMove(self, 0).start()
        self._is_stop = False

    @xarm_is_connected(_type='set')
    def set_servo_attach(self, servo_id=None):
        # assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        # ret = self.arm_cmd.set_brake(servo_id, 0)
        ret = self.motion_enable(servo_id=servo_id, enable=True)
        self.set_state(0)
        return ret

    @xarm_is_connected(_type='set')
    def set_servo_detach(self, servo_id=None):
        """
        :param servo_id: 1-7, 8
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8, 'The value of parameter servo_id can only be 1-8.'
        ret = self.arm_cmd.set_brake(servo_id, 1)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_version(self):
        ret = self.arm_cmd.get_version()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            version = ''.join(list(map(chr, ret[1:])))
            self._version = version[:version.find('\0')]
            ret[0] = 0
            return ret[0], self._version
        else:
            return ret[0], self._version

    @xarm_is_connected(_type='get')
    def get_robot_sn(self):
        ret = self.arm_cmd.get_robot_sn()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            robot_sn = ''.join(list(map(chr, ret[1:])))
            self._robot_sn = robot_sn[:robot_sn.find('\0')]
            ret[0] = 0
            pass
        return ret[0], self._robot_sn

    @xarm_is_connected(_type='set')
    def shutdown_system(self, value=1):
        ret = self.arm_cmd.shutdown_system(value)
        return ret[0]

    def get_is_moving(self):
        self.get_state()
        return self._state == 1

    @xarm_is_connected(_type='get')
    def get_state(self):
        ret = self.arm_cmd.get_state()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._state = ret[1]
            ret[0] = 0
        return ret[0], self._state

    @xarm_is_connected(_type='set')
    def set_state(self, state=0):
        ret = self.arm_cmd.set_state(state)
        if state == 4 and ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            # self._last_position[:6] = self.position
            # self._last_angles = self.angles
            self._sleep_finish_time = 0
        self.get_state()
        if self._state in [3, 4]:
            if self._is_ready:
                logger.info('[set_state], xArm is not ready to move', color='orange')
            self._is_ready = False
        else:
            if not self._is_ready:
                logger.info('[set_state], xArm is ready to move', color='green')
            self._is_ready = True
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_mode(self, mode=0):
        ret = self.arm_cmd.set_mode(mode)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_cmdnum(self):
        ret = self.arm_cmd.get_cmdnum()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._cmd_num = ret[1]
            ret[0] = 0
        return ret[0], self._cmd_num

    @xarm_is_connected(_type='get')
    def get_err_warn_code(self, show=False):
        ret = self.arm_cmd.get_err_code()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._error_code, self._warn_code = ret[1:3]
            ret[0] = 0
        if show:
            pretty_print('*************GetErrorWarnCode, status: {}**************'.format(ret[0]), color='light_blue')
            controller_error = ControllerError(self._error_code)
            controller_warn = ControllerWarn(self._warn_code)
            pretty_print('* ErrorCode: {}, ErrorMsg: {}({})'.format(self._error_code, controller_error.description['cn'], controller_error.description['en']), color='red' if self._error_code != 0 else 'white')
            pretty_print('* WarnCode: {}, WarnMsg: {}({})'.format(self._warn_code, controller_warn.description['cn'], controller_warn.description['en']), color='yellow' if self._warn_code != 0 else 'white')
            pretty_print('*' * 50, color='light_blue')
        return ret[0], [self._error_code, self._warn_code]

    @xarm_is_connected(_type='set')
    def clean_error(self):
        ret = self.arm_cmd.clean_err()
        self.get_state()
        if self._state in [3, 4]:
            if self._is_ready:
                logger.info('[clean_error], xArm is not ready to move', color='orange')
            self._is_ready = False
        else:
            if not self._is_ready:
                logger.info('[clean_error], xArm is ready to move', color='green')
            self._is_ready = True
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_warn(self):
        ret = self.arm_cmd.clean_war()
        return ret[0]

    @xarm_is_connected(_type='set')
    def motion_enable(self, enable=True, servo_id=None):
        assert servo_id is None or (isinstance(servo_id, int) and 1 <= servo_id <= 8)
        if servo_id is None or servo_id == 8:
            ret = self.arm_cmd.motion_en(8, int(enable))
        else:
            ret = self.arm_cmd.motion_en(servo_id, int(enable))
        # if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
        #     self._is_ready = bool(enable)
        self.get_state()
        if self._state in [3, 4]:
            if self._is_ready:
                logger.info('[motion_enable], xArm is not ready to move', color='orange')
            self._is_ready = False
        else:
            if not self._is_ready:
                logger.info('[motion_enable], xArm is ready to move', color='green')
            self._is_ready = True
        return ret[0]

    def reset(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):
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

    @xarm_is_connected(_type='set')
    def set_pause_time(self, sltime, wait=False):
        assert isinstance(sltime, (int, float))
        ret = self.arm_cmd.sleep_instruction(sltime)
        if wait:
            time.sleep(sltime)
        else:
            if time.time() >= self._sleep_finish_time:
                self._sleep_finish_time = time.time() + sltime
            else:
                self._sleep_finish_time += sltime
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_offset(self, offset, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert isinstance(offset, Iterable) and len(offset) >= 6
        tcp_offset = [0] * 6
        for i in range(len(offset)):
            if not offset[i]:
                continue
            if i < 3:
                tcp_offset[i] = offset[i]
            elif i < 6:
                if not is_radian:
                    tcp_offset[i] = offset[i] / RAD_DEGREE
                else:
                    tcp_offset[i] = offset[i]
        ret = self.arm_cmd.set_tcp_offset(tcp_offset)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_jerk(self, jerk):
        ret = self.arm_cmd.set_tcp_jerk(jerk)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_maxacc(self, acc):
        ret = self.arm_cmd.set_tcp_maxacc(acc)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_joint_jerk(self, jerk, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _jerk = jerk
        if not is_radian:
            _jerk = _jerk / RAD_DEGREE
        ret = self.arm_cmd.set_joint_jerk(_jerk)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_joint_maxacc(self, acc, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _acc = acc
        if not is_radian:
            _acc = acc / RAD_DEGREE
        ret = self.arm_cmd.set_joint_maxacc(_acc)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_load(self, weight, center_of_gravity):
        if compare_version(self.version_number, (0, 2, 0)):
            _center_of_gravity = center_of_gravity
        else:
            _center_of_gravity = [item / 1000.0 for item in center_of_gravity]
        ret = self.arm_cmd.set_tcp_load(weight, _center_of_gravity)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_collision_sensitivity(self, value):
        ret = self.arm_cmd.set_collis_sens(value)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_teach_sensitivity(self, value):
        ret = self.arm_cmd.set_teach_sens(value)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_gravity_direction(self, direction):
        ret = self.arm_cmd.set_gravity_dir(direction)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_mount_direction(self, base_tilt_deg, rotation_deg, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        t1 = base_tilt_deg
        t2 = rotation_deg

        if not is_radian:
            t1 = t1 / RAD_DEGREE
            t2 = t2 / RAD_DEGREE

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
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_conf(self):
        ret = self.arm_cmd.clean_conf()
        return ret[0]

    @xarm_is_connected(_type='set')
    def save_conf(self):
        ret = self.arm_cmd.save_conf()
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        assert len(pose) >= 6
        if not input_is_radian:
            pose = [pose[i] if i < 3 else pose[i] / RAD_DEGREE for i in range(6)]
        ret = self.arm_cmd.get_ik(pose)
        angles = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            # angles = [ret[i][0] for i in range(1, 8)]
            angles = [ret[i] for i in range(1, 8)]
            ret[0] = 0
            if not return_is_radian:
                angles = [angle * RAD_DEGREE for angle in angles]
        return ret[0], angles

    @xarm_is_connected(_type='get')
    def get_forward_kinematics(self, angles, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        # assert len(angles) >= 7
        if not input_is_radian:
            angles = [angles[i] / RAD_DEGREE for i in range(len(angles))]

        new_angles = [0] * 7
        for i in range(min(len(angles), 7)):
            new_angles[i] = angles[i]

        ret = self.arm_cmd.get_fk(new_angles)
        pose = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            # pose = [ret[i][0] for i in range(1, 7)]
            pose = [ret[i] for i in range(1, 7)]
            ret[0] = 0
            if not return_is_radian:
                pose = [pose[i] if i < 3 else pose[i] * RAD_DEGREE for i in range(len(pose))]
        return ret[0], pose

    @xarm_is_connected(_type='get')
    def is_tcp_limit(self, pose, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert len(pose) >= 6
        for i in range(6):
            if isinstance(pose[i], str):
                pose[i] = float(pose[i])
            if pose[i] is None:
                pose[i] = self._last_position[i]
            elif i > 2 and not is_radian:
                pose[i] = pose[i] / RAD_DEGREE
        ret = self.arm_cmd.is_tcp_limit(pose)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
            return ret[0], bool(ret[1])
        else:
            return ret[0], None

    @xarm_is_connected(_type='get')
    def is_joint_limit(self, joint, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        # assert len(joint) >= 7
        for i in range(len(joint)):
            if isinstance(joint[i], str):
                joint[i] = float(joint[i])
            if joint[i] is None:
                joint[i] = self._last_angles[i]
            elif not is_radian:
                joint[i] = joint[i] / RAD_DEGREE

        new_angles = [0] * 7
        for i in range(min(len(joint), 7)):
            new_angles[i] = joint[i]

        ret = self.arm_cmd.is_joint_limit(new_angles)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
            return ret[0], bool(ret[1])
        else:
            return ret[0], None

    def _set_params(self, **kwargs):
        is_radian = kwargs.get('is_radian', self._default_is_radian)
        if 'X' in kwargs and isinstance(kwargs['X'], (int, float)):
            self._last_position[0] = kwargs.get('X')
        if 'Y' in kwargs and isinstance(kwargs['Y'], (int, float)):
            self._last_position[1] = kwargs.get('Y')
        if 'Z' in kwargs and isinstance(kwargs['Z'], (int, float)):
            self._last_position[2] = kwargs.get('Z')
        if 'A' in kwargs and isinstance(kwargs['A'], (int, float)):
            self._last_position[3] = kwargs.get('A') if is_radian else kwargs.get('A') / RAD_DEGREE
        if 'B' in kwargs and isinstance(kwargs['B'], (int, float)):
            self._last_position[4] = kwargs.get('B') if is_radian else kwargs.get('B') / RAD_DEGREE
        if 'C' in kwargs and isinstance(kwargs['C'], (int, float)):
            self._last_position[5] = kwargs.get('C') if is_radian else kwargs.get('C') / RAD_DEGREE
        # if 'R' in kwargs and isinstance(kwargs['R'], (int, float)):
        #     self._last_position[6] = kwargs.get('R')
        if 'I' in kwargs and isinstance(kwargs['I'], (int, float)):
            self._last_angles[0] = kwargs.get('I') if is_radian else kwargs.get('I') / RAD_DEGREE
        if 'J' in kwargs and isinstance(kwargs['J'], (int, float)):
            self._last_angles[1] = kwargs.get('J') if is_radian else kwargs.get('J') / RAD_DEGREE
        if 'K' in kwargs and isinstance(kwargs['K'], (int, float)):
            self._last_angles[2] = kwargs.get('K') if is_radian else kwargs.get('K') / RAD_DEGREE
        if 'L' in kwargs and isinstance(kwargs['L'], (int, float)):
            self._last_angles[3] = kwargs.get('L') if is_radian else kwargs.get('L') / RAD_DEGREE
        if 'M' in kwargs and isinstance(kwargs['M'], (int, float)):
            self._last_angles[4] = kwargs.get('M') if is_radian else kwargs.get('M') / RAD_DEGREE
        if 'N' in kwargs and isinstance(kwargs['N'], (int, float)):
            self._last_angles[5] = kwargs.get('N') if is_radian else kwargs.get('N') / RAD_DEGREE
        if 'O' in kwargs and isinstance(kwargs['O'], (int, float)):
            self._last_angles[6] = kwargs.get('O') if is_radian else kwargs.get('O') / RAD_DEGREE

        if 'F' in kwargs and isinstance(kwargs['F'], (int, float)):
            self._last_tcp_speed = kwargs.get('F')
            self._last_tcp_speed = min(max(self._last_tcp_speed, self._min_tcp_speed), self._max_tcp_speed)
        if 'Q' in kwargs and isinstance(kwargs['Q'], (int, float)):
            self._last_tcp_acc = kwargs.get('Q')
            self._last_tcp_acc = min(max(self._last_tcp_acc, self._min_tcp_acc), self._max_tcp_acc)
        if 'F2' in kwargs and isinstance(kwargs['F2'], (int, float)):
            self._last_joint_speed = kwargs.get('F2')
            if not is_radian:
                self._last_joint_speed /= RAD_DEGREE
            self._last_joint_speed = min(max(self._last_joint_speed, self._min_joint_speed), self._max_joint_speed)
        if 'Q2' in kwargs and isinstance(kwargs['Q2'], (int, float)):
            self._last_joint_acc = kwargs.get('Q2')
            if not is_radian:
                self._last_joint_acc /= RAD_DEGREE
            self._last_joint_acc = min(max(self._last_joint_acc, self._min_joint_acc), self._max_joint_acc)
        if 'T' in kwargs and isinstance(kwargs['T'], (int, float)):
            self._mvtime = kwargs.get('T')
        if 'LIMIT_VELO' in kwargs and isinstance(kwargs['LIMIT_VELO'], (list, tuple)) \
                and len(kwargs['LIMIT_VELO']) == 2 and isinstance(kwargs['LIMIT_VELO'][0], (int, float)) \
                and isinstance(kwargs['LIMIT_VELO'][1], (int, float)) \
                and kwargs['LIMIT_VELO'][0] <= kwargs['LIMIT_VELO'][1]:
            self._min_tcp_speed, self._max_tcp_speed = kwargs.get('LIMIT_VELO')
        if 'LIMIT_ACC' in kwargs and isinstance(kwargs['LIMIT_ACC'], (list, tuple)) \
                and len(kwargs['LIMIT_ACC']) == 2 and isinstance(kwargs['LIMIT_ACC'][0], (int, float)) \
                and isinstance(kwargs['LIMIT_ACC'][1], (int, float)) \
                and kwargs['LIMIT_ACC'][0] <= kwargs['LIMIT_ACC'][1]:
            self._min_tcp_acc, self._max_tcp_acc = kwargs.get('LIMIT_ACC')

    def _get_params(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if is_radian:
            return {
                'lastPosition': self._last_position,
                'lastAngles': self._last_angles,
                'mvvelo': self._last_tcp_speed,
                'mvacc': self._last_tcp_acc,
                'angle_mvvelo': self._last_joint_speed,
                'angle_mvacc': self._last_joint_acc,
                'mvtime': self._mvtime,
                'LIMIT_VELO': [self._min_tcp_speed, self._max_tcp_speed],
                'LIMIT_ACC': [self._min_tcp_acc, self._max_tcp_acc],
                'LIMIT_ANGLE_VELO': [self._min_joint_speed, self._max_joint_speed],
                'LIMIT_ANGLE_ACC': [self._min_joint_acc, self._max_joint_acc],
            }
        else:
            return {
                'lastPosition': [self._last_position[i] * RAD_DEGREE if 2 < i < 6 else self._last_position[i] for i in range(len(self._last_position))],
                'lastAngles': [angle * RAD_DEGREE for angle in self._last_angles],
                'mvvelo': int(self._last_tcp_speed),
                'mvacc': int(self._last_tcp_acc),
                'angle_mvvelo': int(self._last_joint_speed * RAD_DEGREE),
                'angle_mvacc': int(self._last_joint_acc * RAD_DEGREE),
                'mvtime': self._mvtime,
                'LIMIT_VELO': list(map(int, [self._min_tcp_speed, self._max_tcp_speed])),
                'LIMIT_ACC': list(map(int, [self._min_tcp_acc, self._max_tcp_acc])),
                'LIMIT_ANGLE_VELO': list(map(int, [self._min_joint_speed * RAD_DEGREE, self._max_joint_speed * RAD_DEGREE])),
                'LIMIT_ANGLE_ACC': list(map(int, [self._min_joint_acc * RAD_DEGREE, self._max_joint_acc * RAD_DEGREE])),
            }

    def emergency_stop(self):
        start_time = time.time()
        while self.state != 4 and time.time() - start_time < 3:
            self.set_state(4)
            time.sleep(0.1)
        self._is_stop = True
        start_time = time.time()
        self.motion_enable(enable=True)
        while self.state in [0, 3, 4] and time.time() - start_time < 3:
            self.set_state(0)
            time.sleep(0.1)
        self._sleep_finish_time = 0

    def send_cmd_async(self, command, timeout=None):
        pass

    def send_cmd_sync(self, command=None):
        if command is None:
            return 0
        if command.lower() == 'help':
            return 0, {
                'G1': 'set_position(MoveLine): G1 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}',
                'G4': 'set_pause_time: G4 V{sltime(second)}',
                'G7': 'set_servo_angle: G7 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)} F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}',
                'G8': 'move_gohome: G8 F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}',
                'G9': 'set_position(MoveArcLine): G9 X{x} Y{y} Z{z} A{roll} B{pitch(° or rad)} C{yaw(° or rad)} R{radius(mm)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}',
                'H1': 'get_version: H1',
                'H11': 'motion_enable: H11 S{servo_id} V{enable}',
                'H12': 'set_state: H12 V{state}',
                'H13': 'get_state: H13',
                'H14': 'get_cmdnum: H14',
                'H15': 'get_err_warn_code: H15',
                'H16': 'clean_error: H16',
                'H17': 'clean_warn: H17',
                'H18': 'set_servo_attach/set_servo_detach: H18 S{servo_id} V{1: enable(detach), 0: disable(attach)}',
                'H31': 'set_tcp_jerk: H31 V{jerk(mm/s^3)}',
                'H32': 'set_tcp_maxacc: H32 V{maxacc(mm/s^2)}',
                'H33': 'set_joint_jerk: H33 V{jerk(°/s^3 or rad/s^3)}',
                'H34': 'set_joint_maxacc: H34 {maxacc(°/s^2 or rad/s^2)}',
                'H39': 'clean_conf: H39',
                'H40': 'save_conf: H40',
                'H41': 'get_position: H41',
                'H42': 'get_servo_angle: H42',
                'H43': 'get_inverse_kinematics: H43 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}',
                'H44': 'get_forward_kinematics: H44 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}',
                'H45': 'is_joint_limit: H45 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}',
                'H46': 'is_tcp_limit: H46 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}',
                # 'H101': '(Danger, please do not use) set_servo_addr_16: H101 S{servo_id(1-7)} A{addr} V{value}',
                # 'H102': '(Danger, please do not use) get_servo_addr_16: H102 S{servo_id(1-7)} A{addr}',
                # 'H103': '(Danger, please do not use) set_servo_addr_32: H103 S{servo_id(1-7)} A{addr} V{value}',
                # 'H104': '(Danger, please do not use) get_servo_addr_32: H104 S{servo_id(1-7)} A{addr}',
                # 'H105': '(Danger, please do not use) set_servo_zero: H105 S{servo_id(1-7)}',
                'H106': 'get_servo_debug_msg: H106',
            }
        num = parse.gcode_get_chint(command, 'G')
        if num == 1:  # G1 xarm_move_line ex: G1 X300 Y0 Z100 A-180 B0 C0 F100 Q50 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvpose = parse.gcode_get_mvcarts(command)
            wait = parse.gcode_get_wait(command)
            ret = self.set_position(*mvpose, radius=-1, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, wait=wait)
            # ret = self.set_position(*mvpose, radius=-1, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 4:  # G4 xarm_sleep_cmd ex: G4 V1
            sltime = parse.gcode_get_mvtime(command)
            ret = self.set_pause_time(sltime)
        elif num == 7:  # G7 xarm_move_joint ex: G7 I11 J22 K33 L44 M-56 N67 O45 F50 Q30 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvjoint = parse.gcode_get_mvjoints(command)
            wait = parse.gcode_get_wait(command)
            ret = self.set_servo_angle(angle=mvjoint, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, wait=wait)
            # ret = self.set_servo_angle(angle=mvjoint, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 8:  # G8 xarm_move_gohome ex: G8 F100 Q40 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            wait = parse.gcode_get_wait(command)
            ret = self.move_gohome(speed=mvvelo, mvacc=mvacc, mvtime=mvtime, wait=wait)
            # ret = self.move_gohome(speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 9:  # G9 xarm_move_arc_line ex: G9 X300 Y0 Z100 A-180 B0 C0 R10 F100 Q50 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvpose = parse.gcode_get_mvcarts(command)
            mvradii = parse.gcode_get_mvradii(command)
            wait = parse.gcode_get_wait(command)
            if mvradii is None:
                mvradii = 0
            ret = self.set_position(*mvpose, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, radius=mvradii, wait=wait)
            # ret = self.set_position(*mvpose, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, radius=mvradii, is_radian=False)
        else:
            num = parse.gcode_get_chint(command, 'H')
            if num == 1:  # H0 H1 get_version ex: H0
                ret = self.get_version()
            elif num == 11:  # H11 motion_enable ex: H11 V1
                value = parse.gcode_get_chint(command, 'V')
                servo_id = parse.gcode_get_chint(command, 'S')
                if value == -1:
                    value = True
                if servo_id < 1:
                    servo_id = None
                ret = self.motion_enable(enable=value, servo_id=servo_id)
            elif num == 12:  # H12 set_state ex: H12 V0
                value = parse.gcode_get_chint(command, 'V')
                ret = self.set_state(value)
            elif num == 13:  # H13 get_state ex: H13
                ret = self.get_state()
            elif num == 14:  # H14 get_cmd_num ex: H14
                ret = self.get_cmdnum()
            elif num == 15:  # H15 get_error_warn_code ex: H15
                ret = self.get_err_warn_code()
            elif num == 16:  # H16 clean_error ex: H16
                ret = self.clean_error()
            elif num == 17:  # H17 clean_warn ex: H17
                ret = self.clean_warn()
            elif num == 18:  # H18 set_brake ex: H18 V0 S{servo_id}
                value = parse.gcode_get_chint(command, 'V')
                servo_id = parse.gcode_get_servo(command)
                if value == -1:
                    value = 0
                if value == 0:
                    ret = self.set_servo_attach(servo_id=servo_id)
                else:
                    ret = self.set_servo_detach(servo_id=servo_id)
            elif num == 31:  # H31 set_tcp_jerk ex: H31 V30
                value = parse.gcode_get_value(command)
                ret = self.set_tcp_jerk(value)
            elif num == 32:  # H32 set_tcp_maxacc ex: H32 V500
                value = parse.gcode_get_value(command)
                ret = self.set_tcp_maxacc(value)
            elif num == 33:  # H33 set_joint_jerk ex: H33 V30
                value = parse.gcode_get_value(command)
                ret = self.set_joint_jerk(value)
            elif num == 34:  # H34 set_joint_maxacc ex: H34 V100
                value = parse.gcode_get_value(command)
                ret = self.set_joint_maxacc(value)
            elif num == 39:  # H39 clean_conf ex: H39
                ret = self.clean_conf()
            elif num == 40:  # H40 save_conf ex: H40
                ret = self.save_conf()
            elif num == 41:  # H41 get_position ex: H41
                # value = parse.gcode_get_chint(command, 'V')
                # if value == -1:
                #     value = 0
                # ret = self.get_position(is_radian=value == 0)
                ret = self.get_position()
            elif num == 42:  # H42 get_servo_angle ex: H42
                # value = parse.gcode_get_chint(command, 'V')
                # if value == -1:
                #     value = 0
                # ret = self.get_servo_angle(is_radian=value == 0)
                ret = self.get_servo_angle()
            elif num == 43:  # H43 get_ik ex: H43 X100 Y0 Z100 A90 B90 C100
                pose = parse.gcode_get_mvcarts(command)
                ret = self.get_inverse_kinematics(pose, input_is_radian=False, return_is_radian=False)
            elif num == 44:  # H44 get_fk ex: H44 I11 J22 K33 L44 M-56 N67 O45
                joint = parse.gcode_get_mvjoints(command)
                ret = self.get_forward_kinematics(joint, input_is_radian=False, return_is_radian=False)
            elif num == 45:  # H45 is_joint_limit ex: H45 I11 J22 K33 L44 M-56 N67 O45
                joint = parse.gcode_get_mvjoints(command)
                ret = self.is_joint_limit(joint, is_radian=False)
            elif num == 46:  # H46 is_tcp_limit ex: H46 X100 Y0 Z100 A90 B90 C100
                pose = parse.gcode_get_mvcarts(command)
                ret = self.is_tcp_limit(pose, is_radian=False)
            elif num == 101:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                value = parse.gcode_get_value(command)
                ret = self.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)
            elif num == 102:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                ret = self.get_servo_addr_16(servo_id=servo_id, addr=addr)
            elif num == 103:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                value = parse.gcode_get_value(command)
                ret = self.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)
            elif num == 104:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                ret = self.get_servo_addr_32(servo_id=servo_id, addr=addr)
            elif num == 105:
                servo_id = parse.gcode_get_servo(command)
                ret = self.set_servo_zero(servo_id=servo_id)
            elif num == 106:
                ret = self.get_servo_debug_msg()
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
        return ret
