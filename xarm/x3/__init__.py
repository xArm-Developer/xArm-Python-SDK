#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import re
import os
import math
import time
import warnings
import threading
from collections import Iterable
from ..core.comm import SerialPort, SocketPort
from ..core.config.x_config import XCONF
from ..core.wrapper import UxbusCmdSer, UxbusCmdTcp
from ..core.utils import convert
from ..core.utils.log import logger, pretty_print
from ..core.config.x_code import ControllerWarn, ControllerError, ControllerErrorCodeMap, ControllerWarnCodeMap
from .gripper import Gripper
from .gpio import GPIO
from .servo import Servo
from .events import *
from .record import Record
from .parse import GcodeParser
from .code import APIState
from .utils import xarm_is_connected, xarm_is_ready, xarm_is_pause, compare_time, compare_version
try:
    from ..tools.blockly_tool import BlocklyTool
except:
    print('import BlocklyTool module failed')
    BlocklyTool = None


gcode_p = GcodeParser()


class XArm(Gripper, Servo, GPIO, Events, Record):
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
        self._max_cmd_num = kwargs.get('max_cmdnum', 256)
        if not isinstance(self._max_cmd_num, int):
            self._max_cmd_num = 256
        self._max_cmd_num = min(XCONF.MAX_CMD_NUM, self._max_cmd_num)
        self._check_robot_sn = kwargs.get('check_robot_sn', False)
        self._check_is_ready = kwargs.get('check_is_ready', True)
        self._check_is_pause = kwargs.get('check_is_pause', True)
        self._timed_comm = kwargs.get('timed_comm', True)
        self._timed_comm_interval = kwargs.get('timed_comm_interval', 180)
        self._timed_comm_t = None
        self._timed_comm_t_alive = False

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
        self._world_offset = [0] * 6
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
        axis = kwargs.get('axis', self._arm_axis)
        if axis in [5, 6, 7]:
            self._arm_axis = axis
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
        self._first_report_over = False
        self._default_is_radian = is_radian

        self._sleep_finish_time = time.time()
        self._is_old_protocol = False

        self._major_version_number = 0  # 固件主版本号
        self._minor_version_number = 0  # 固件次版本号
        self._revision_version_number = 0  # 固件修正版本号

        self._temperatures = [0, 0, 0, 0, 0, 0, 0]

        self._is_set_move = False
        self._cond_pause = threading.Condition()

        self._version_ge_1_2_11 = False  # 固件是否大于等于1.2.11，用于get_reduced_states
        self._realtime_tcp_speed = 0
        self._realtime_joint_speeds = [0, 0, 0, 0, 0, 0, 0]

        self._count = -1

        Events.__init__(self)
        if not do_not_open:
            self.connect()

    def check_is_pause(self):
        if self._check_is_pause:
            if self.state == 3 and self._enable_report:
                with self._cond_pause:
                    self._cond_pause.wait()

    @property
    def version_is_ge_1_2_11(self):
        if not self._version:
            self.get_version()
            if self._version and isinstance(self._version, str):
                pattern = re.compile(r'.*[vV](\d+)\.(\d+)\.(\d+)')
                m = re.match(pattern, self._version)
                if m:
                    (self._major_version_number,
                     self._minor_version_number,
                     self._revision_version_number) = map(int, m.groups())
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
        return self._major_version_number > 1 or (
            self._major_version_number == 1 and self._minor_version_number > 2) or (
            self._major_version_number == 1 and self._minor_version_number == 2 and self._revision_version_number >= 11)

    @property
    def realtime_tcp_speed(self):
        return self._realtime_tcp_speed

    @property
    def realtime_joint_speeds(self):
        return [speed if self._default_is_radian else math.degrees(speed) for speed in self._realtime_joint_speeds]

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
        return [math.degrees(self._position[i]) if 2 < i < 6 and not self._default_is_radian
                else self._position[i] for i in range(len(self._position))]

    @property
    def tcp_jerk(self):
        return self._tcp_jerk

    @property
    def tcp_speed_limit(self):
        return [self._min_tcp_speed, self._max_tcp_speed]

    @property
    def tcp_acc_limit(self):
        return [self._min_tcp_acc, self._max_tcp_acc]

    @property
    def last_used_position(self):
        return [math.degrees(self._last_position[i]) if 2 < i < 6 and not self._default_is_radian
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
        return [angle if self._default_is_radian else math.degrees(angle) for angle in self._angles]

    @property
    def joint_jerk(self):
        return self._joint_jerk if self._default_is_radian else math.degrees(self._joint_jerk)

    @property
    def joint_speed_limit(self):
        limit = [self._min_joint_speed, self._max_joint_speed]
        if not self._default_is_radian:
            limit = [math.degrees(i) for i in limit]
        return limit

    @property
    def joint_acc_limit(self):
        limit = [self._min_joint_acc, self._max_joint_acc]
        if not self._default_is_radian:
            limit = [math.degrees(i) for i in limit]
        return limit

    @property
    def last_used_angles(self):
        return [angle if self._default_is_radian else math.degrees(angle) for angle in self._last_angles]

    @property
    def last_used_joint_speed(self):
        return self._last_joint_speed if self._default_is_radian else math.degrees(self._last_joint_speed)

    @property
    def last_used_joint_acc(self):
        return self._last_joint_acc if self._default_is_radian else math.degrees(self._last_joint_acc)

    @property
    def position_offset(self):
        return [math.degrees(self._position_offset[i]) if 2 < i < 6 and not self._default_is_radian
                else self._position_offset[i] for i in range(len(self._position_offset))]

    @property
    def world_offset(self):
        return [math.degrees(self._world_offset[i]) if 2 < i < 6 and not self._default_is_radian
                else self._world_offset[i] for i in range(len(self._world_offset))]

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
    def temperatures(self):
        return self._temperatures

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

    def _timed_comm_thread(self):
        self._timed_comm_t_alive = True
        while self.connected and self._timed_comm_t_alive:
            try:
                self.get_cmdnum()
            except:
                pass
            count = self._timed_comm_interval * 10
            while count > 0:
                count -= 1
                time.sleep(0.1)
                if not self._timed_comm_t_alive or not self.connected:
                    break

    def connect(self, port=None, baudrate=None, timeout=None, axis=None):
        if self.connected:
            return
        if axis in [5, 6, 7]:
            self._arm_axis = axis
        self._is_ready = True
        self._port = port if port is not None else self._port
        self._baudrate = baudrate if baudrate is not None else self._baudrate
        self._timeout = timeout if timeout is not None else self._timeout
        if not self._port:
            raise Exception('can not connect to port/ip {}'.format(self._port))
        if self._timed_comm_t is not None:
            try:
                self._timed_comm_t_alive = False
                self._timed_comm_t.join()
                self._timed_comm_t = None
            except:
                pass
        self._is_first_report = True
        self._first_report_over = False
        if isinstance(self._port, (str, bytes)):
            if self._port == 'localhost' or re.match(
                    r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$",
                    self._port):
                self._stream = SocketPort(self._port, XCONF.SocketConf.TCP_CONTROL_PORT,
                                          heartbeat=self._enable_heartbeat,
                                          buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE)
                if not self.connected:
                    raise Exception('connect socket failed')

                try:
                    if self._timed_comm:
                        self._timed_comm_t = threading.Thread(target=self._timed_comm_thread, daemon=True)
                        self._timed_comm_t.start()
                except:
                    pass

                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdTcp(self._stream)
                self._stream_type = 'socket'
                self._stream_report = None

                try:
                    self._connect_report()
                except:
                    self._stream_report = None

                self._check_version()

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
                if self._enable_report:
                    self._report_thread = threading.Thread(target=self._auto_get_report_thread, daemon=True)
                    self._report_thread.start()
                    self._report_connect_changed_callback(True, True)
                else:
                    self._report_connect_changed_callback(True, False)
                self._check_version()

    def _check_version(self):
        self._version = None
        self._robot_sn = None
        try:
            count = 2
            while not self._version and count:
                self.get_version()
                time.sleep(0.1)
                count -= 1
            if self._version and isinstance(self._version, str):
                pattern = re.compile(r'.*[vV](\d+)\.(\d+)\.(\d+)')
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
            if self._check_robot_sn:
                count = 2
                while not self._robot_sn and count and self.warn_code == 0:
                    self.get_robot_sn()
                    self.get_err_warn_code()
                    if not self._robot_sn and self.warn_code == 0 and count:
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
                                             buffer_size=1024
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

    def _report_temperature_changed_callback(self):
        if REPORT_TEMPERATURE_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_TEMPERATURE_CHANGED_ID]:
                try:
                    callback({
                        'temperatures': self.temperatures,
                    })
                except Exception as e:
                    logger.error('temperature changed callback: {}'.format(e))

    def _report_count_changed_callback(self):
        if REPORT_COUNT_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_COUNT_CHANGED_ID]:
                try:
                    callback({
                        'count': self._count
                    })
                except Exception as e:
                    logger.error('count changed callback: {}'.format(e))

    def _report_location_callback(self):
        if REPORT_LOCATION_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[REPORT_LOCATION_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self.position.copy()
                if item['joints']:
                    ret['joints'] = self.angles.copy()
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
                    ret['cartesian'] = self.position.copy()
                if item['joints']:
                    ret['joints'] = self.angles.copy()
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
                if error_code != self._error_code:
                    self._error_code = error_code
                    if self._error_code != 0:
                        pretty_print('Error, code: {}'.format(self._error_code), color='red')
                    else:
                        pretty_print('Error had clean', color='blue')
                if warn_code != self._warn_code:
                    self._warn_code = warn_code
                    if self._warn_code != 0:
                        pretty_print('Warn, code: {}'.format(self._warn_code), color='yellow')
                    else:
                        pretty_print('Warnning had clean', color='blue')
                self._report_error_warn_changed_callback()
                logger.info('OnReport -> err={}, warn={}, state={}, cmdnum={}, mtbrake={}, mtable={}'.format(
                    error_code, warn_code, state, cmd_num, mtbrake, mtable
                ))
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
                    # if self._is_ready:
                    #     pretty_print('[report], xArm is not ready to move', color='red')
                    self._is_ready = False
                else:
                    # if not self._is_ready:
                    #     pretty_print('[report], xArm is ready to move', color='green')
                    self._is_ready = True
            else:
                self._is_ready = False
            self._is_first_report = False

            self._error_code = error_code
            self._warn_code = warn_code
            self.arm_cmd.has_err_warn = error_code != 0 or warn_code != 0
            self._state = state
            if self.state != 3:
                with self._cond_pause:
                    self._cond_pause.notifyAll()
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
            self._first_report_over = True

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
            # if state != self._state or mode != self._mode:
            #     print('mode: {}, state={}, time={}'.format(mode, state, time.time()))
            cmd_num = convert.bytes_to_u16(rx_data[5:7])
            angles = convert.bytes_to_fp32s(rx_data[7:7 * 4 + 7], 7)
            pose = convert.bytes_to_fp32s(rx_data[35:6 * 4 + 35], 6)
            torque = convert.bytes_to_fp32s(rx_data[59:7 * 4 + 59], 7)
            mtbrake, mtable, error_code, warn_code = rx_data[87:91]
            if (error_code != 0 and error_code not in ControllerErrorCodeMap.keys()) \
                    or (warn_code != 0 and warn_code not in ControllerWarnCodeMap.keys()):
                self._stream_report.close()
                print('DataException，ErrorCode: {}, WarnCode: {}, try reconnect'.format(error_code, warn_code))
                return
            pose_offset = convert.bytes_to_fp32s(rx_data[91:6 * 4 + 91], 6)
            tcp_load = convert.bytes_to_fp32s(rx_data[115:4 * 4 + 115], 4)
            collis_sens, teach_sens = rx_data[131:133]
            self._gravity_direction = convert.bytes_to_fp32s(rx_data[133:3*4 + 133], 3)

            # print('torque: {}'.format(torque))
            # print('tcp_load: {}'.format(tcp_load))
            # print('collis_sens: {}, teach_sens: {}'.format(collis_sens, teach_sens))

            if error_code != self._error_code or warn_code != self._warn_code:
                if error_code != self._error_code:
                    self._error_code = error_code
                    if self._error_code != 0:
                        pretty_print('ControllerError, code: {}'.format(self._error_code), color='red')
                    else:
                        pretty_print('ControllerError had clean', color='blue')
                if warn_code != self._warn_code:
                    self._warn_code = warn_code
                    if self._warn_code != 0:
                        pretty_print('ControllerWarning, code: {}'.format(self._warn_code), color='yellow')
                    else:
                        pretty_print('ControllerWarning had clean', color='blue')
                self._report_error_warn_changed_callback()
                logger.info('OnReport -> err={}, warn={}, state={}, cmdnum={}, mtbrake={}, mtable={}, mode={}'.format(
                    error_code, warn_code, state, cmd_num, mtbrake, mtable, mode
                ))
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
                    # if self._is_ready:
                    #     pretty_print('[report], xArm is not ready to move', color='red')
                    self._is_ready = False
                else:
                    # if not self._is_ready:
                    #     pretty_print('[report], xArm is ready to move', color='green')
                    self._is_ready = True
            else:
                self._is_ready = False
            self._is_first_report = False

            self._error_code = error_code
            self._warn_code = warn_code
            self.arm_cmd.has_err_warn = error_code != 0 or warn_code != 0
            self._state = state
            if self.state != 3:
                with self._cond_pause:
                    self._cond_pause.notifyAll()
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
            self._first_report_over = True

            size = convert.bytes_to_u32(rx_data[0:4])
            if size >= 252:
                temperatures = list(map(int, rx_data[245:252]))
                if temperatures != self.temperatures:
                    self._temperatures = temperatures
                    self._report_temperature_changed_callback()
            if size >= 284:
                speeds = convert.bytes_to_fp32s(rx_data[252:8 * 4 + 252], 8)
                self._realtime_tcp_speed = speeds[0]
                self._realtime_joint_speeds = speeds[1:]
                # print(speeds[0], speeds[1:])
            if size >= 288:
                count = convert.bytes_to_u32(data[284:288])
                # print(count, data[284:288])
                if self._count != -1 and count != self._count:
                    self._count = count
                    self._report_count_changed_callback()
                self._count = count
            if size >= 312:
                world_offset = convert.bytes_to_fp32s(rx_data[288:6 * 4 + 288], 6)
                for i in range(len(world_offset)):
                    if i < 3:
                        world_offset[i] = float('{:.3f}'.format(world_offset[i]))
                    else:
                        world_offset[i] = float('{:.6f}'.format(world_offset[i]))
                if math.inf not in world_offset and -math.inf not in world_offset and not (10 <= self._error_code <= 17):
                    self._world_offset = world_offset

        main_socket_connected = self._stream and self._stream.connected
        report_socket_connected = self._stream_report and self._stream_report.connected
        buffer = b''
        size = 0
        while self._stream and self._stream.connected:
            try:
                if not self._stream_report or not self._stream_report.connected:
                    self.get_err_warn_code()
                    if report_socket_connected:
                        report_socket_connected = False
                        self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                    self._connect_report()
                    buffer = b''
                    size = 0
                    continue
                if not report_socket_connected:
                    report_socket_connected = True
                    self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                rx_data = self._stream_report.read()
                if rx_data != -1:
                    buffer += rx_data
                    if len(buffer) < 4:
                        continue
                    if size == 0:
                        size = convert.bytes_to_u32(buffer[0:4])
                    if len(buffer) < size:
                        continue
                    if size >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                        if size >= XCONF.SocketConf.TCP_REPORT_RICH_BUF_SIZE:
                            if size == 233 and len(buffer) == 245:
                                data = buffer[:245]
                                buffer = buffer[245:]
                            else:
                                data = buffer[:size]
                                buffer = buffer[size:]
                            __handle_report_rich(data)
                        else:
                            data = buffer[:size]
                            buffer = buffer[size:]
                            __handle_report_normal(data)
                # if rx_data != -1 and len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                #     # if len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                #     #     __handle_report_normal(rx_data)
                #     if len(rx_data) >= XCONF.SocketConf.TCP_REPORT_RICH_BUF_SIZE:
                #         __handle_report_rich(rx_data)
                #     else:
                #         __handle_report_normal(rx_data)
            except Exception as e:
                logger.error(e)
                self._connect_report()
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

                if self.state != 3:
                    with self._cond_pause:
                        self._cond_pause.notifyAll()
                if cmd_num != self._cmd_num:
                    self._report_cmdnum_changed_callback()
                if state != self._state:
                    self._report_state_changed_callback()
                if state == 4:
                    # if self._is_ready:
                    #     pretty_print('[report], xArm is not ready to move', color='red')
                    self._is_ready = False
                else:
                    # if not self._is_ready:
                    #     pretty_print('[report], xArm is ready to move', color='green')
                    self._is_ready = True
                if error_code != self._error_code or warn_code != self._warn_code:
                    self._report_error_warn_changed_callback()
                elif not self._only_report_err_warn_changed and (self._error_code != 0 or self._warn_code != 0):
                    self._report_error_warn_changed_callback()

                self._report_location_callback()
                self._report_callback()

                if self._cmd_num >= self._max_cmd_num:
                    time.sleep(1)

                time.sleep(0.1)
            except:
                pass
        self._report_connect_changed_callback(False, False)
        logger.debug('get report thread stopped')

    def disconnect(self):
        try:
            self._stream.close()
        except:
            pass
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
        with self._cond_pause:
            self._cond_pause.notifyAll()

    def _sync_tcp(self, index=None):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_angles = self._angles
        if index is None:
            self._last_position = self._position
        elif isinstance(index, int) and 0 <= index < 6:
            self._last_position[index] = self._position[index]
        print('=============sync_tcp: index={}'.format(index))

    def _sync_joints(self, index=None):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_position = self._position
        if index is None:
            self._last_angles = self._angles
        elif isinstance(index, int) and 0 <= index < 7:
            self._last_angles[index] = self._angles[index]
        print('=============sync_joint: index={}'.format(index))

    def _sync(self):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_position = self._position
        self._last_angles = self._angles
        print('=============sync_all')

    class _WaitMove:
        def __init__(self, owner, timeout):
            self.owner = owner
            # self.timeout = timeout if timeout is not None else 10
            self.timeout = timeout if timeout is not None else -1
            self.timer = None
            self.is_timeout = False

        def start(self):
            if self.timeout > 0:
                if self.owner._sleep_finish_time - time.time() > 0:
                    self.timeout += self.owner._sleep_finish_time - time.time()
                self.timer = threading.Timer(self.timeout, self.timeout_cb)
                self.timer.setDaemon(True)
                self.timer.start()
            self.check_stop_move()

        def check_stop_move(self):
            # base_joint_pos = self.owner.angles.copy()
            time.sleep(0.1)
            count = 0
            while not self.is_timeout and not self.owner._is_stop and self.owner.connected and not self.owner.has_error:
                if self.owner.state == 4:
                    self.owner._sleep_finish_time = 0
                    break
                if time.time() < self.owner._sleep_finish_time:
                    time.sleep(0.02)
                    count = 0
                    continue
                if self.owner.state == 3:
                    time.sleep(0.02)
                    count = 0
                    continue
                # if self.owner.angles == base_joint_pos or self.owner.state != 1:
                if self.owner.state != 1:
                    count += 1
                    if count >= 10:
                        break
                else:
                    # base_joint_pos = self.owner._angles.copy()
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
        return ret[0], [math.degrees(self._position[i]) if 2 < i < 6 and not is_radian else self._position[i] for i in
                        range(len(self._position))]

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
                # if value == limit[0] or value == limit[0] - 2 * math.pi:
                #     return False
                # else:
                #     logger.info('API -> set_position -> ret={}, i={}, value={}'.format(APIState.OUT_OF_RANGE, i, value))
                #     return True
            if value < limit[0] or value > limit[1]:
                logger.info('API -> set_position -> ret={}, i={} value={}'.format(APIState.OUT_OF_RANGE, i, value))
                return True
        return False

    def _wait_until_cmdnum_lt_max(self):
        if not self._check_cmdnum_limit:
            return
        self._is_stop = False
        while self.cmd_num >= self._max_cmd_num:
            if not self.connected:
                return APIState.NOT_CONNECTED
            elif not self.ready:
                return APIState.NOT_READY
            elif self._is_stop:
                return APIState.EMERGENCY_STOP
            elif self.has_error:
                return
            time.sleep(0.1)

    @xarm_is_ready(_type='set')
    @xarm_is_pause(_type='set')
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None,
                     wait=False, timeout=None, **kwargs):
        ret = self._wait_until_cmdnum_lt_max()
        if ret is not None:
            logger.info('API -> set_position -> ret={}'.format(ret))
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
                        if self._is_out_of_tcp_range(self._last_position[i] + math.radians(value), i):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] += math.radians(value)
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
                        if self._is_out_of_tcp_range(math.radians(value), i):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] = math.radians(value)
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
        else:
            ret = self.arm_cmd.move_line(self._last_position, self._last_tcp_speed, self._last_tcp_acc, self._mvtime)
        logger.info('API -> set_position -> ret={}, pos={}, radius={}, velo={}, acc={}'.format(
            ret[0], self._last_position, radius, self._last_tcp_speed, self._last_tcp_acc
        ))
        self._is_set_move = True
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                warnings.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
                return APIState.HAS_ERROR if self.error_code != 0 else APIState.HAS_WARN if self.warn_code != 0 else APIState.NORMAL
        if ret[0] < 0 and not self.get_is_moving():
            self._last_position = last_used_position
            self._last_tcp_speed = last_used_tcp_speed
            self._last_tcp_acc = last_used_tcp_acc
        return ret[0]

    @xarm_is_ready(_type='set')
    @xarm_is_pause(_type='set')
    def set_tool_position(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0,
                          speed=None, mvacc=None, mvtime=None, is_radian=None,
                          wait=False, timeout=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        last_used_tcp_speed = self._last_tcp_speed
        last_used_tcp_acc = self._last_tcp_acc
        mvpose = [x, y, z, roll, pitch, yaw]
        if not is_radian:
            mvpose = [x, y, z, math.radians(roll), math.radians(pitch), math.radians(yaw)]
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

        ret = self.arm_cmd.move_line_tool(mvpose, self._last_tcp_speed, self._last_tcp_acc, self._mvtime)
        logger.info('API -> set_tool_position -> ret={}, pos={}, velo={}, acc={}'.format(
            ret[0], mvpose, self._last_tcp_speed, self._last_tcp_acc
        ))
        self._is_set_move = True
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                warnings.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
                return APIState.HAS_ERROR if self.error_code != 0 else APIState.HAS_WARN if self.warn_code != 0 else APIState.NORMAL
        if ret[0] < 0 and not self.get_is_moving():
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
            return ret[0], list(map(lambda x: x if is_radian else math.degrees(x), self._angles))
        else:
            return ret[0], self._angles[servo_id-1] if is_radian else math.degrees(self._angles[servo_id-1])

    def _is_out_of_joint_range(self, angle, i):
        if not self._check_joint_limit or self._stream_type != 'socket' or not self._enable_report:
            return False
        joint_limit = XCONF.Robot.JOINT_LIMITS.get(self.axis).get(self.device_type, [])
        if i < len(joint_limit):
            angle_range = joint_limit[i]
            if angle < angle_range[0] or angle > angle_range[1]:
                logger.info('API -> set_servo_angle -> ret={}, i={} value={}'.format(APIState.OUT_OF_RANGE, i, angle))
                return True
        return False

    @xarm_is_ready(_type='set')
    @xarm_is_pause(_type='set')
    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=None, wait=False, timeout=None, **kwargs):
        assert ((servo_id is None or servo_id == 8) and isinstance(angle, Iterable)) \
            or (1 <= servo_id <= 7 and angle is not None and not isinstance(angle, Iterable)), \
            'param servo_id or angle error'
        ret = self._wait_until_cmdnum_lt_max()
        if ret is not None:
            logger.info('API -> set_servo_angle -> ret={}'.format(ret))
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
                        if self._is_out_of_joint_range(self._last_angles[i] + math.radians(value), i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] += math.radians(value)
                else:
                    if is_radian:
                        if self._is_out_of_joint_range(value, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] = value
                    else:
                        if self._is_out_of_joint_range(math.radians(value), i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] = math.radians(value)
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
                    if self._is_out_of_joint_range(self._last_angles[servo_id - 1] + math.radians(angle), servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] += math.radians(angle)
            else:
                if is_radian:
                    if self._is_out_of_joint_range(angle, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] = angle
                else:
                    if self._is_out_of_joint_range(math.radians(angle), servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] = math.radians(angle)

        if speed is not None:
            if isinstance(speed, str):
                if speed.isdigit():
                    speed = float(speed)
                else:
                    speed = self._last_joint_speed if is_radian else math.degrees(self._last_joint_speed)
            if not is_radian:
                speed = math.radians(speed)
            self._last_joint_speed = min(max(speed, self._min_joint_speed), self._max_joint_speed)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                if mvvelo.isdigit():
                    mvvelo = float(mvvelo)
                else:
                    mvvelo = self._last_joint_speed if is_radian else math.degrees(self._last_joint_speed)
            if not is_radian:
                mvvelo = math.radians(mvvelo)
            self._last_joint_speed = min(max(mvvelo, self._min_joint_speed), self._max_joint_speed)
        if mvacc is not None:
            if isinstance(mvacc, str):
                if mvacc.isdigit():
                    mvacc = float(mvacc)
                else:
                    mvacc = self._last_joint_acc if is_radian else math.degrees(self._last_joint_acc)
            if not is_radian:
                mvacc = math.radians(mvacc)
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
        logger.info('API -> set_servo_angle -> ret={}, angles={}, velo={}, acc={}'.format(
            ret[0], self._last_angles, self._last_joint_speed, self._last_joint_acc
        ))
        self._is_set_move = True
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                warnings.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
                return APIState.HAS_ERROR if self.error_code != 0 else APIState.HAS_WARN if self.warn_code != 0 else APIState.NORMAL
        if ret[0] < 0 and not self.get_is_moving():
            self._last_angles = last_used_angle
            self._last_joint_speed = last_used_joint_speed
            self._last_joint_acc = last_used_joint_acc
        return ret[0]

    @xarm_is_ready(_type='set')
    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _angles = [angle if is_radian else math.radians(angle) for angle in angles]
        for i in range(self.axis):
            if self._is_out_of_joint_range(_angles[i], i):
                return APIState.OUT_OF_RANGE
        while len(_angles) < 7:
            _angles.append(0)
        _speed = self._last_joint_speed if speed is None else speed
        _mvacc = self._last_joint_acc if mvacc is None else mvacc
        _mvtime = self._mvtime if mvtime is None else mvtime
        ret = self.arm_cmd.move_servoj(_angles, _speed, _mvacc, _mvtime)
        logger.info('API -> set_servo_angle_j -> ret={}, angles={}, velo={}, acc={}'.format(
            ret[0], _angles, _speed, _mvacc
        ))
        self._is_set_move = True
        return ret[0]

    @xarm_is_ready(_type='set')
    def set_servo_cartesian(self, mvpose, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        assert len(mvpose) >= 6
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if not is_radian:
            pose = [mvpose[i] if i < 3 else math.radians(mvpose[i]) for i in range(6)]
        else:
            pose = mvpose
        _speed = self.last_used_tcp_speed if speed is None else speed
        _mvacc = self.last_used_tcp_acc if mvacc is None else mvacc
        _mvtime = self._mvtime if mvtime is None else mvtime

        ret = self.arm_cmd.move_servo_cartesian(pose, _speed, _mvacc, _mvtime)
        logger.info('API -> set_servo_cartisian -> ret={}, pose={}, velo={}, acc={}'.format(
            ret[0], pose, _speed, _mvacc
        ))
        self._is_set_move = True
        return ret[0]

    @xarm_is_ready(_type='set')
    @xarm_is_pause(_type='set')
    def move_circle(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        ret = self._wait_until_cmdnum_lt_max()
        if ret is not None:
            logger.info('API -> move_circle -> ret={}'.format(ret))
            return ret
        last_used_tcp_speed = self._last_tcp_speed
        last_used_tcp_acc = self._last_tcp_acc
        is_radian = self._default_is_radian if is_radian is None else is_radian
        pose_1 = []
        pose_2 = []
        for i in range(6):
            pose_1.append(pose1[i] if i < 3 or is_radian else math.radians(pose1[i]))
            pose_2.append(pose2[i] if i < 3 or is_radian else math.radians(pose2[i]))
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
        logger.info('API -> move_circle -> ret={}, pos1={}, pos2={}, percent={}%, velo={}, acc={}'.format(
            ret[0], pose_1, pose_2, percent, self._last_tcp_speed, self._last_tcp_acc
        ))
        self._is_set_move = True

        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                print('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
                return APIState.HAS_ERROR if self.error_code != 0 else APIState.HAS_WARN if self.warn_code != 0 else APIState.NORMAL
        if ret[0] < 0 and not self.get_is_moving():
            self._last_tcp_speed = last_used_tcp_speed
            self._last_tcp_acc = last_used_tcp_acc
        return ret[0]

    @xarm_is_ready(_type='set')
    @xarm_is_pause(_type='set')
    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        # if speed is None:
        #     speed = 0.8726646259971648  # 50 °/s
        # else:
        #     if not is_radian:
        #         speed = math.radians(speed)
        # if mvacc is None:
        #     mvacc = 17.453292519943297  # 1000 °/s^2
        # else:
        #     if not is_radian:
        #         mvacc = math.radians(mvacc)
        # if mvtime is None:
        #     mvtime = 0

        if speed is not None:
            if isinstance(speed, str):
                if speed.isdigit():
                    speed = float(speed)
                else:
                    speed = self._last_joint_speed if is_radian else math.degrees(self._last_joint_speed)
            if not is_radian:
                speed = math.radians(speed)
            self._last_joint_speed = min(max(speed, self._min_joint_speed), self._max_joint_speed)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                if mvvelo.isdigit():
                    mvvelo = float(mvvelo)
                else:
                    mvvelo = self._last_joint_speed if is_radian else math.degrees(self._last_joint_speed)
            if not is_radian:
                mvvelo = math.radians(mvvelo)
            self._last_joint_speed = min(max(mvvelo, self._min_joint_speed), self._max_joint_speed)
        if mvacc is not None:
            if isinstance(mvacc, str):
                if mvacc.isdigit():
                    mvacc = float(mvacc)
                else:
                    mvacc = self._last_joint_acc if is_radian else math.degrees(self._last_joint_acc)
            if not is_radian:
                mvacc = math.radians(mvacc)
            self._last_joint_acc = min(max(mvacc, self._min_joint_acc), self._max_joint_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                if mvacc.isdigit():
                    mvtime = float(mvtime)
                else:
                    mvtime = self._mvtime
            self._mvtime = mvtime

        ret = self.arm_cmd.move_gohome(self._last_joint_speed, self._last_joint_acc, self._mvtime)
        logger.info('API -> move_gohome -> ret={}, velo={}, acc={}'.format(
            ret[0], self._last_joint_speed, self._last_joint_acc
        ))
        self._is_set_move = True
        if ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            pass
            # self._last_position = [201.5, 0, 140.5, -3.1415926, 0, 0]
            # self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                warnings.warn('if you want to wait, please enable report')
            else:
                self._is_stop = False
                self._WaitMove(self, timeout).start()
                self._is_stop = False
                return APIState.HAS_ERROR if self.error_code != 0 else APIState.HAS_WARN if self.warn_code != 0 else APIState.NORMAL
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
        logger.info('move_arc_lines--begin')
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
        logger.info('move_arc_lines--end')
        if wait:
            self._WaitMove(self, 0).start()
        self._is_stop = False

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
        logger.info('API -> set_servo_detach -> ret={}'.format(ret[0]))
        self._sync()
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

    @xarm_is_connected(_type='get')
    def check_verification(self):
        ret = self.arm_cmd.check_verification()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def shutdown_system(self, value=1):
        ret = self.arm_cmd.shutdown_system(value)
        logger.info('API -> shutdown_system -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_reduced_mode(self, on_off):
        ret = self.arm_cmd.set_reduced_mode(on_off)
        logger.info('API -> set_reduced_mode -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_reduced_max_tcp_speed(self, speed):
        ret = self.arm_cmd.set_reduced_linespeed(speed)
        logger.info('API -> set_reduced_linespeed -> ret={}, speed={}'.format(ret[0], speed))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_reduced_max_joint_speed(self, speed, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        speed = speed
        if not is_radian:
            speed = math.radians(speed)
        ret = self.arm_cmd.set_reduced_jointspeed(speed)
        logger.info('API -> set_reduced_linespeed -> ret={}, speed={}'.format(ret[0], speed))
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_reduced_mode(self):
        ret = self.arm_cmd.get_reduced_mode()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_reduced_states(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_reduced_states(79 if self.version_is_ge_1_2_11 else 21)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
            if not is_radian:
                ret[4] = round(math.degrees(ret[4]), 1)
                if self.version_is_ge_1_2_11:
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
        logger.info('API -> set_reduced_tcp_boundary -> ret={}, boundary={}'.format(ret[0], limits))
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
        logger.info('API -> set_reduced_joint_range -> ret={}, boundary={}'.format(ret[0], limits))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_fense_mode(self, on_off):
        ret = self.arm_cmd.set_fense_on(on_off)
        logger.info('API -> set_fense_mode -> ret={}, on={}'.format(ret[0], on_off))
        return ret

    @xarm_is_connected(_type='set')
    def set_collision_rebound(self, on_off):
        ret = self.arm_cmd.set_collis_reb(on_off)
        logger.info('API -> set_collision_rebound -> ret={}, on={}'.format(ret[0], on_off))
        return ret

    @xarm_is_connected(_type='set')
    def set_timer(self, secs_later, tid, fun_code, param1=0, param2=0):
        ret = self.arm_cmd.set_timer(secs_later, tid, fun_code, param1, param2)
        return ret[0]

    @xarm_is_connected(_type='set')
    def cancel_timer(self, tid):
        ret = self.arm_cmd.cancel_timer(tid)
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_world_offset(self, offset, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert isinstance(offset, Iterable) and len(offset) >= 6
        world_offset = [0] * 6
        for i in range(len(offset)):
            if not offset[i]:
                continue
            if i < 3:
                world_offset[i] = offset[i]
            elif i < 6:
                if not is_radian:
                    world_offset[i] = math.radians(offset[i])
                else:
                    world_offset[i] = offset[i]
        ret = self.arm_cmd.set_world_offset(world_offset)
        logger.info('API -> set_world_offset -> ret={}, offset={}'.format(ret[0], world_offset))
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
        _state = self._state
        ret = self.arm_cmd.set_state(state)
        if state == 4 and ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            # self._last_position[:6] = self.position
            # self._last_angles = self.angles
            self._sleep_finish_time = 0
            # self._is_sync = False
        self.get_state()
        if _state != self._state:
            self._report_state_changed_callback()
        if self._state != 3:
            with self._cond_pause:
                self._cond_pause.notifyAll()
        if self._state in [4]:
            if self._is_ready:
                pretty_print('[set_state], xArm is not ready to move', color='red')
            self._is_ready = False
        else:
            if not self._is_ready:
                pretty_print('[set_state], xArm is ready to move', color='green')
            self._is_ready = True
        logger.info('set_state({}), ret={}, state={}'.format(state, ret[0], self._state))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_mode(self, mode=0):
        ret = self.arm_cmd.set_mode(mode)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
        logger.info('API -> set_mode -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_cmdnum(self):
        ret = self.arm_cmd.get_cmdnum()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            if ret[1] != self._cmd_num:
                self._report_cmdnum_changed_callback()
            self._cmd_num = ret[1]
            ret[0] = 0
        return ret[0], self._cmd_num

    @xarm_is_connected(_type='get')
    def get_err_warn_code(self, show=False, lang='en'):
        ret = self.arm_cmd.get_err_code()
        lang = lang if lang == 'cn' else 'en'
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._error_code, self._warn_code = ret[1:3]
            ret[0] = 0
        if show:
            pretty_print('************* {}, {}: {} **************'.format(
                         '获取控制器错误警告码' if lang == 'cn' else 'GetErrorWarnCode',
                         '状态' if lang == 'cn' else 'Status',
                         ret[0]), color='light_blue')
            controller_error = ControllerError(self._error_code, status=0)
            controller_warn = ControllerWarn(self._warn_code, status=0)
            pretty_print('* {}: {}, {}: {}'.format(
                '错误码' if lang == 'cn' else 'ErrorCode',
                controller_error.code,
                '信息' if lang == 'cn' else 'Info',
                controller_error.title[lang]),
                         color='red' if self._error_code != 0 else 'white')
            pretty_print('* {}: {}, {}: {}'.format(
                '警告码' if lang == 'cn' else 'WarnCode',
                controller_warn.code,
                '信息' if lang == 'cn' else 'Info',
                controller_warn.title[lang]),
                         color='yellow' if self._warn_code != 0 else 'white')
            pretty_print('*' * 50, color='light_blue')
        return ret[0], [self._error_code, self._warn_code]

    @xarm_is_connected(_type='set')
    def clean_error(self):
        ret = self.arm_cmd.clean_err()
        self.get_state()
        if self._state in [4]:
            if self._is_ready:
                pretty_print('[clean_error], xArm is not ready to move', color='red')
            self._is_ready = False
        else:
            if not self._is_ready:
                pretty_print('[clean_error], xArm is ready to move', color='green')
            self._is_ready = True
        logger.info('API -> clean_error -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_warn(self):
        ret = self.arm_cmd.clean_war()
        logger.info('API -> clean_warn -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def motion_enable(self, enable=True, servo_id=None):
        assert servo_id is None or (isinstance(servo_id, int) and 1 <= servo_id <= 8)
        if servo_id is None or servo_id == 8:
            ret = self.arm_cmd.motion_en(8, int(enable))
        else:
            ret = self.arm_cmd.motion_en(servo_id, int(enable))
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._is_ready = bool(enable)
        self.get_state()
        if self._state in [4]:
            if self._is_ready:
                pretty_print('[motion_enable], xArm is not ready to move', color='red')
            self._is_ready = False
        else:
            if not self._is_ready:
                pretty_print('[motion_enable], xArm is ready to move', color='green')
            self._is_ready = True
        logger.info('API -> motion_enable -> ret={}'.format(ret[0]))
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

    @xarm_is_ready(_type='set')
    def set_joint_torque(self, jnt_taus):
        ret = self.arm_cmd.set_servot(jnt_taus)
        logger.info('API -> set_joint_torque -> ret={}, jnt_taus={}'.format(ret[0], jnt_taus))
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_joint_torque(self, servo_id=None):
        ret = self.arm_cmd.get_joint_tau()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 7:
            self._joints_torque = [float('{:.6f}'.format(ret[i])) for i in range(1, 8)]
            ret[0] = 0
        if servo_id is None or servo_id == 8 or len(self._joints_torque) < servo_id:
            return ret[0], list(self._joints_torque)
        else:
            return ret[0], self._joints_torque[servo_id - 1]

    @xarm_is_connected(_type='get')
    def get_safe_level(self):
        ret = self.arm_cmd.get_safe_level()
        # if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
        #     self._state = ret[1]
        #     ret[0] = 0
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_safe_level(self, level=4):
        ret = self.arm_cmd.set_safe_level(level)
        logger.info('API -> set_safe_level -> ret={}, level={}'.format(ret[0], level))
        return ret[0]

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
        logger.info('API -> set_pause_time -> ret={}, sltime={}'.format(ret[0], sltime))
        return ret[0]

    def set_sleep_time(self, sltime, wait=False):
        return self.set_pause_time(sltime, wait)

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
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
                    tcp_offset[i] = math.radians(offset[i])
                else:
                    tcp_offset[i] = offset[i]
        ret = self.arm_cmd.set_tcp_offset(tcp_offset)
        logger.info('API -> set_tcp_offset -> ret={}, offset={}'.format(ret[0], tcp_offset))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_jerk(self, jerk):
        ret = self.arm_cmd.set_tcp_jerk(jerk)
        logger.info('API -> set_tcp_jerk -> ret={}, jerk={}'.format(ret[0], jerk))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_maxacc(self, acc):
        ret = self.arm_cmd.set_tcp_maxacc(acc)
        logger.info('API -> set_tcp_maxacc -> ret={}, maxacc={}'.format(ret[0], acc))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_joint_jerk(self, jerk, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _jerk = jerk
        if not is_radian:
            _jerk = math.radians(_jerk)
        ret = self.arm_cmd.set_joint_jerk(_jerk)
        logger.info('API -> set_joint_jerk -> ret={}, jerk={}'.format(ret[0], _jerk))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_joint_maxacc(self, acc, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _acc = acc
        if not is_radian:
            _acc = math.radians(acc)
        ret = self.arm_cmd.set_joint_maxacc(_acc)
        logger.info('API -> set_joint_maxacc -> ret={}, maxacc={}'.format(ret[0], acc))
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_tcp_load(self, weight, center_of_gravity):
        if compare_version(self.version_number, (0, 2, 0)):
            _center_of_gravity = center_of_gravity
        else:
            _center_of_gravity = [item / 1000.0 for item in center_of_gravity]
        ret = self.arm_cmd.set_tcp_load(weight, _center_of_gravity)
        logger.info('API -> set_tcp_load -> ret={}, weight={}, center={}'.format(ret[0], weight, _center_of_gravity))
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_collision_sensitivity(self, value):
        assert isinstance(value, int) and 0 <= value <= 5
        ret = self.arm_cmd.set_collis_sens(value)
        logger.info('API -> set_collision_sensitivity -> ret={}, sensitivity={}'.format(ret[0], value))
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_teach_sensitivity(self, value):
        assert isinstance(value, int) and 1 <= value <= 5
        ret = self.arm_cmd.set_teach_sens(value)
        logger.info('API -> set_teach_sensitivity -> ret={}, sensitivity={}'.format(ret[0], value))
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
    def set_gravity_direction(self, direction):
        ret = self.arm_cmd.set_gravity_dir(direction[:3])
        logger.info('API -> set_gravity_direction -> ret={}, direction={}'.format(ret[0], direction))
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_pause(_type='set')
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
        logger.info('API -> set_mount_direction -> ret={}, tilt={}, rotation={}, direction={}'.format(ret[0], base_tilt_deg, rotation_deg, g_new))
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_conf(self):
        ret = self.arm_cmd.clean_conf()
        logger.info('API -> clean_conf -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def save_conf(self):
        ret = self.arm_cmd.save_conf()
        logger.info('API -> save_conf -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        assert len(pose) >= 6
        if not input_is_radian:
            pose = [pose[i] if i < 3 else math.radians(pose[i]) for i in range(6)]
        ret = self.arm_cmd.get_ik(pose)
        angles = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            # angles = [ret[i][0] for i in range(1, 8)]
            angles = [ret[i] for i in range(1, 8)]
            ret[0] = 0
            if not return_is_radian:
                angles = [math.degrees(angle) for angle in angles]
        return ret[0], angles

    @xarm_is_connected(_type='get')
    def get_forward_kinematics(self, angles, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        # assert len(angles) >= 7
        if not input_is_radian:
            angles = [math.radians(angles[i]) for i in range(len(angles))]

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
                pose = [pose[i] if i < 3 else math.degrees(pose[i]) for i in range(len(pose))]
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
                pose[i] = math.radians(pose[i])
        ret = self.arm_cmd.is_tcp_limit(pose)
        logger.info('API -> is_tcp_limit -> ret={}, limit={}'.format(ret[0], ret[1]))
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
                joint[i] = math.radians(joint[i])

        new_angles = [0] * 7
        for i in range(min(len(joint), 7)):
            new_angles[i] = joint[i]

        ret = self.arm_cmd.is_joint_limit(new_angles)
        logger.info('API -> is_joint_limit -> ret={}, limit={}'.format(ret[0], ret[1]))
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
            self._last_position[3] = kwargs.get('A') if is_radian else math.radians(kwargs.get('A'))
        if 'B' in kwargs and isinstance(kwargs['B'], (int, float)):
            self._last_position[4] = kwargs.get('B') if is_radian else math.radians(kwargs.get('B'))
        if 'C' in kwargs and isinstance(kwargs['C'], (int, float)):
            self._last_position[5] = kwargs.get('C') if is_radian else math.radians(kwargs.get('C'))
        # if 'R' in kwargs and isinstance(kwargs['R'], (int, float)):
        #     self._last_position[6] = kwargs.get('R')
        if 'I' in kwargs and isinstance(kwargs['I'], (int, float)):
            self._last_angles[0] = kwargs.get('I') if is_radian else math.radians(kwargs.get('I'))
        if 'J' in kwargs and isinstance(kwargs['J'], (int, float)):
            self._last_angles[1] = kwargs.get('J') if is_radian else math.radians(kwargs.get('J'))
        if 'K' in kwargs and isinstance(kwargs['K'], (int, float)):
            self._last_angles[2] = kwargs.get('K') if is_radian else math.radians(kwargs.get('K'))
        if 'L' in kwargs and isinstance(kwargs['L'], (int, float)):
            self._last_angles[3] = kwargs.get('L') if is_radian else math.radians(kwargs.get('L'))
        if 'M' in kwargs and isinstance(kwargs['M'], (int, float)):
            self._last_angles[4] = kwargs.get('M') if is_radian else math.radians(kwargs.get('M'))
        if 'N' in kwargs and isinstance(kwargs['N'], (int, float)):
            self._last_angles[5] = kwargs.get('N') if is_radian else math.radians(kwargs.get('N'))
        if 'O' in kwargs and isinstance(kwargs['O'], (int, float)):
            self._last_angles[6] = kwargs.get('O') if is_radian else math.radians(kwargs.get('O'))

        if 'F' in kwargs and isinstance(kwargs['F'], (int, float)):
            self._last_tcp_speed = kwargs.get('F')
            self._last_tcp_speed = min(max(self._last_tcp_speed, self._min_tcp_speed), self._max_tcp_speed)
        if 'Q' in kwargs and isinstance(kwargs['Q'], (int, float)):
            self._last_tcp_acc = kwargs.get('Q')
            self._last_tcp_acc = min(max(self._last_tcp_acc, self._min_tcp_acc), self._max_tcp_acc)
        if 'F2' in kwargs and isinstance(kwargs['F2'], (int, float)):
            self._last_joint_speed = kwargs.get('F2')
            if not is_radian:
                self._last_joint_speed = math.radians(self._last_joint_speed)
            self._last_joint_speed = min(max(self._last_joint_speed, self._min_joint_speed), self._max_joint_speed)
        if 'Q2' in kwargs and isinstance(kwargs['Q2'], (int, float)):
            self._last_joint_acc = kwargs.get('Q2')
            if not is_radian:
                self._last_joint_acc = math.radians(self._last_joint_acc)
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
                'tcpJerk': self._tcp_jerk,
                'jointJerk': self._joint_jerk,
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
                'lastPosition': [math.degrees(self._last_position[i]) if 2 < i < 6 else self._last_position[i] for i in range(len(self._last_position))],
                'lastAngles': [math.degrees(angle) for angle in self._last_angles],
                'mvvelo': round(self._last_tcp_speed),
                'mvacc': round(self._last_tcp_acc),
                'tcpJerk': round(self._tcp_jerk),
                'jointJerk': round(math.degrees(self._joint_jerk)),
                'angle_mvvelo': round(math.degrees(self._last_joint_speed)),
                'angle_mvacc': round(math.degrees(self._last_joint_acc)),
                'mvtime': self._mvtime,
                'LIMIT_VELO': list(map(round, [self._min_tcp_speed, self._max_tcp_speed])),
                'LIMIT_ACC': list(map(round, [self._min_tcp_acc, self._max_tcp_acc])),
                'LIMIT_ANGLE_VELO': list(map(round, [math.degrees(self._min_joint_speed), math.degrees(self._max_joint_speed)])),
                'LIMIT_ANGLE_ACC': list(map(round, [math.degrees(self._min_joint_acc), math.degrees(self._max_joint_acc)])),
            }

    def emergency_stop(self):
        start_time = time.time()
        logger.info('emergency_stop--begin')
        while self.state != 4 and time.time() - start_time < 3:
            self.set_state(4)
            time.sleep(0.1)
        self._is_stop = True
        # self.set_state(0)
        # if not self._is_ready:
        #     start_time = time.time()
        #     self.motion_enable(enable=True)
        #     while self.state in [0, 3, 4] and time.time() - start_time < 3:
        #         ret = self.set_state(0)
        #         if ret == 1:
        #             break
        #         time.sleep(0.1)
        self._sleep_finish_time = 0
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
            elif num == 10:  # H10 shutdown_system, ex: H10 V{}
                value = gcode_p.get_int_value(command, default=0)
                ret = self.shutdown_system(value)
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
            succeed = blockly_tool.to_python(arm=self, **kwargs)
            if succeed:
                times = kwargs.get('times', 1)
                for i in range(times):
                    exec(blockly_tool.codes, {'arm': self})
                return APIState.NORMAL
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
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
        logger.info('API -> reload_dynamics -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_counter_reset(self):
        ret = self.arm_cmd.cnter_reset()
        # if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
        #     ret[0] = 0
        logger.info('API -> set_counter_reset -> ret={}'.format(ret[0]))
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_counter_increase(self, val=1):
        ret = self.arm_cmd.cnter_plus()
        # if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
        #     ret[0] = 0
        logger.info('API -> set_counter_increase -> ret={}'.format(ret[0]))
        return ret[0]


